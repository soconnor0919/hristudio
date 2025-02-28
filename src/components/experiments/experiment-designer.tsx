"use client";

import { useState, useCallback, useRef } from "react";
import ReactFlow, {
  Background,
  Controls,
  MiniMap,
  type Node,
  type Edge,
  type Connection,
  type NodeChange,
  type EdgeChange,
  applyNodeChanges,
  applyEdgeChanges,
  ReactFlowProvider,
  Panel,
} from "reactflow";
import { motion, AnimatePresence } from "framer-motion";
import { type Step } from "~/lib/experiments/types";
import { BUILT_IN_ACTIONS, getPluginActions, type ActionConfig } from "~/lib/experiments/plugin-actions";
import { Card } from "~/components/ui/card";
import { ScrollArea } from "~/components/ui/scroll-area";
import { Tabs, TabsList, TabsTrigger, TabsContent } from "~/components/ui/tabs";
import { ActionNode } from "./nodes/action-node";
import { FlowEdge } from "./edges/flow-edge";
import { ActionItem } from "./action-item";
import { cn } from "~/lib/utils";
import { Button } from "~/components/ui/button";
import { ChevronLeft, ChevronRight, Undo, Redo, ZoomIn, ZoomOut } from "lucide-react";
import { api } from "~/trpc/react";
import "reactflow/dist/style.css";

const nodeTypes = {
  action: ActionNode,
};

const edgeTypes = {
  default: FlowEdge,
};

interface ExperimentDesignerProps {
  className?: string;
  defaultSteps?: Step[];
  onChange?: (steps: Step[]) => void;
  readOnly?: boolean;
}

export function ExperimentDesigner({
  className,
  defaultSteps = [],
  onChange,
  readOnly = false,
}: ExperimentDesignerProps) {
  const [sidebarOpen, setSidebarOpen] = useState(true);
  const [selectedNode, setSelectedNode] = useState<Node | null>(null);
  const reactFlowWrapper = useRef<HTMLDivElement>(null);
  const [reactFlowInstance, setReactFlowInstance] = useState<any>(null);

  // Get available plugins
  const { data: plugins } = api.pluginStore.getPlugins.useQuery();
  const { data: installedPlugins } = api.pluginStore.getInstalledPlugins.useQuery();
  const installedPluginIds = installedPlugins?.map(p => p.robotId) ?? [];

  // Get available actions from installed plugins
  const installedPluginActions = plugins
    ? getPluginActions(plugins.filter(p => installedPluginIds.includes(p.robotId)))
    : [];

  // Combine built-in actions with plugin actions
  const availableActions = [...BUILT_IN_ACTIONS, ...installedPluginActions];

  // History management
  const [history, setHistory] = useState<Step[][]>([defaultSteps]);
  const [historyIndex, setHistoryIndex] = useState(0);

  const addToHistory = useCallback((newSteps: Step[]) => {
    setHistory(prev => {
      const newHistory = prev.slice(0, historyIndex + 1);
      return [...newHistory, newSteps];
    });
    setHistoryIndex(prev => prev + 1);
  }, [historyIndex]);

  const undo = useCallback(() => {
    if (historyIndex > 0) {
      setHistoryIndex(prev => prev - 1);
      const prevSteps = history[historyIndex - 1]!;
      setSteps(prevSteps);
      onChange?.(prevSteps);
    }
  }, [history, historyIndex, onChange]);

  const redo = useCallback(() => {
    if (historyIndex < history.length - 1) {
      setHistoryIndex(prev => prev + 1);
      const nextSteps = history[historyIndex + 1]!;
      setSteps(nextSteps);
      onChange?.(nextSteps);
    }
  }, [history, historyIndex, onChange]);

  // Convert steps to nodes and edges
  const initialNodes: Node[] = defaultSteps.flatMap((step, stepIndex) =>
    step.actions.map((action, actionIndex) => ({
      id: action.id,
      type: "action",
      position: { x: stepIndex * 250, y: actionIndex * 150 },
      data: {
        type: action.type,
        parameters: action.parameters,
        onChange: (parameters: Record<string, any>) => {
          const newSteps = [...steps];
          const stepIndex = newSteps.findIndex(s =>
            s.actions.some(a => a.id === action.id)
          );
          const actionIndex = stepIndex !== -1
            ? newSteps[stepIndex]!.actions.findIndex(
              a => a.id === action.id
            )
            : -1;

          if (
            stepIndex !== -1 &&
            actionIndex !== -1 &&
            newSteps[stepIndex]?.actions[actionIndex]
          ) {
            const step = newSteps[stepIndex]!;
            const updatedAction = { ...step.actions[actionIndex]!, parameters };
            step.actions[actionIndex] = updatedAction;
            setSteps(newSteps);
            addToHistory(newSteps);
            onChange?.(newSteps);
          }
        },
      },
    }))
  );

  const initialEdges: Edge[] = defaultSteps.flatMap((step, stepIndex) =>
    step.actions.slice(0, -1).map((action, actionIndex) => ({
      id: `${action.id}-${step.actions[actionIndex + 1]?.id}`,
      source: action.id,
      target: step.actions[actionIndex + 1]?.id ?? "",
      type: "default",
      animated: true,
    }))
  );

  const [nodes, setNodes] = useState<Node[]>(initialNodes);
  const [edges, setEdges] = useState<Edge[]>(initialEdges);
  const [steps, setSteps] = useState<Step[]>(defaultSteps);

  const onNodesChange = useCallback(
    (changes: NodeChange[]) => {
      setNodes((nds) => applyNodeChanges(changes, nds));
      const selectedChange = changes.find(
        (change) => change.type === "select"
      );
      if (selectedChange) {
        const node = nodes.find((n) => n.id === selectedChange.id);
        setSelectedNode(selectedChange.selected ? node : null);
      }
    },
    [nodes]
  );

  const onEdgesChange = useCallback(
    (changes: EdgeChange[]) => {
      setEdges((eds) => applyEdgeChanges(changes, eds));
    },
    []
  );

  const onConnect = useCallback(
    (connection: Connection) => {
      if (!connection.source || !connection.target) return;

      const sourceNode = nodes.find((n) => n.id === connection.source);
      const targetNode = nodes.find((n) => n.id === connection.target);

      if (sourceNode && targetNode) {
        const newSteps = [...steps];
        const sourceStep = newSteps.find((s) =>
          s.actions.some((a) => a.id === sourceNode.id)
        );
        const targetStep = newSteps.find((s) =>
          s.actions.some((a) => a.id === targetNode.id)
        );

        if (sourceStep && targetStep) {
          const sourceAction = sourceStep.actions.find(
            (a) => a.id === sourceNode.id
          );
          const targetAction = targetStep.actions.find(
            (a) => a.id === targetNode.id
          );
          if (sourceAction && targetAction) {
            const targetStepIndex = newSteps.indexOf(targetStep);
            newSteps[targetStepIndex]!.actions = targetStep.actions.filter(
              (a) => a.id !== targetAction.id
            );
            const sourceStepIndex = newSteps.indexOf(sourceStep);
            const sourceActionIndex = sourceStep.actions.indexOf(sourceAction);
            newSteps[sourceStepIndex]!.actions.splice(
              sourceActionIndex + 1,
              0,
              targetAction
            );
          }
        }
        setSteps(newSteps);
        addToHistory(newSteps);
        onChange?.(newSteps);
      }
    },
    [nodes, steps, onChange, addToHistory]
  );

  const onDragOver = useCallback((event: React.DragEvent) => {
    event.preventDefault();
    event.dataTransfer.dropEffect = "move";
  }, []);

  const onDrop = useCallback(
    (event: React.DragEvent) => {
      event.preventDefault();

      if (!reactFlowWrapper.current || !reactFlowInstance) return;

      const type = event.dataTransfer.getData("application/reactflow");
      if (!type) return;

      const position = reactFlowInstance.screenToFlowPosition({
        x: event.clientX,
        y: event.clientY,
      });

      const actionConfig = availableActions.find((a) => a.type === type);
      if (!actionConfig) return;

      const newAction = {
        id: crypto.randomUUID(),
        type: actionConfig.type,
        parameters: { ...actionConfig.defaultParameters },
        order: 0,
      };

      const newNode: Node = {
        id: newAction.id,
        type: "action",
        position,
        data: {
          type: actionConfig.type,
          parameters: newAction.parameters,
          onChange: (parameters: Record<string, any>) => {
            const newSteps = [...steps];
            const stepIndex = newSteps.findIndex((s) =>
              s.actions.some((a) => a.id === newAction.id)
            );
            const actionIndex = stepIndex !== -1
              ? newSteps[stepIndex]!.actions.findIndex(
                a => a.id === newAction.id
              )
              : -1;

            if (
              stepIndex !== -1 &&
              actionIndex !== -1 &&
              newSteps[stepIndex]?.actions[actionIndex]
            ) {
              const step = newSteps[stepIndex]!;
              const updatedAction = { ...step.actions[actionIndex]!, parameters };
              step.actions[actionIndex] = updatedAction;
              setSteps(newSteps);
              addToHistory(newSteps);
              onChange?.(newSteps);
            }
          },
        },
      };

      setNodes((nds) => [...nds, newNode]);

      const newStep: Step = {
        id: crypto.randomUUID(),
        title: `Step ${steps.length + 1}`,
        actions: [newAction],
        order: steps.length,
      };

      setSteps((s) => [...s, newStep]);
      addToHistory([...steps, newStep]);
      onChange?.([...steps, newStep]);
    },
    [steps, onChange, reactFlowInstance, addToHistory, availableActions]
  );

  // Group actions by source
  const groupedActions = availableActions.reduce((acc, action) => {
    const source = action.pluginId ?
      plugins?.find(p => p.robotId === action.pluginId)?.name ?? action.pluginId :
      'Built-in Actions';

    if (!acc[source]) {
      acc[source] = [];
    }
    acc[source].push(action);
    return acc;
  }, {} as Record<string, ActionConfig[]>);

  return (
    <div className={cn("relative flex h-full", className)}>
      <AnimatePresence>
        {sidebarOpen && (
          <motion.div
            initial={{ x: -320, opacity: 0 }}
            animate={{ x: 0, opacity: 1 }}
            exit={{ x: -320, opacity: 0 }}
            transition={{ type: "spring", damping: 20, stiffness: 300 }}
            className="absolute inset-y-0 left-0 z-30 w-80 overflow-hidden"
          >
            <Card className="flex h-full flex-col rounded-lg border shadow-2xl">
              <Tabs defaultValue="actions" className="flex h-full flex-col">
                <div className="flex h-12 shrink-0 items-center justify-between border-b px-4">
                  <TabsList>
                    <TabsTrigger value="actions">Actions</TabsTrigger>
                    <TabsTrigger value="properties">Properties</TabsTrigger>
                  </TabsList>
                  <Button
                    variant="ghost"
                    size="icon"
                    className="h-8 w-8"
                    onClick={() => setSidebarOpen(false)}
                  >
                    <ChevronLeft className="h-4 w-4" />
                  </Button>
                </div>
                <TabsContent value="actions" className="flex-1 p-0">
                  <ScrollArea className="h-full">
                    <div className="space-y-6 p-4">
                      {Object.entries(groupedActions).map(([source, actions]) => (
                        <div key={source} className="space-y-2">
                          <h3 className="px-2 text-sm font-medium text-muted-foreground">
                            {source}
                          </h3>
                          <div className="space-y-2">
                            {actions.map((action) => (
                              <ActionItem
                                key={action.pluginId ? `${action.pluginId}:${action.type}` : action.type}
                                type={action.type}
                                title={action.title}
                                description={action.description}
                                icon={action.icon}
                                draggable
                                onDragStart={(event) => {
                                  event.dataTransfer.setData(
                                    "application/reactflow",
                                    action.type
                                  );
                                  event.dataTransfer.effectAllowed = "move";
                                }}
                              />
                            ))}
                          </div>
                        </div>
                      ))}
                    </div>
                  </ScrollArea>
                </TabsContent>
                <TabsContent value="properties" className="flex-1 p-0">
                  <ScrollArea className="h-full">
                    <div className="p-4">
                      {selectedNode ? (
                        <div className="space-y-4">
                          <h3 className="font-medium">
                            {availableActions.find((a) => a.type === selectedNode.data.type)?.title}
                          </h3>
                          <pre className="rounded-lg bg-muted p-4 text-xs">
                            {JSON.stringify(selectedNode.data.parameters, null, 2)}
                          </pre>
                        </div>
                      ) : (
                        <div className="text-sm text-muted-foreground">
                          Select a node to view its properties
                        </div>
                      )}
                    </div>
                  </ScrollArea>
                </TabsContent>
              </Tabs>
            </Card>
          </motion.div>
        )}
      </AnimatePresence>

      {!sidebarOpen && (
        <Button
          variant="outline"
          size="icon"
          className="absolute left-4 top-4 z-20"
          onClick={() => setSidebarOpen(true)}
        >
          <ChevronRight className="h-4 w-4" />
        </Button>
      )}

      <div
        ref={reactFlowWrapper}
        className={cn(
          "relative h-full flex-1 transition-[margin] duration-200 ease-in-out",
          sidebarOpen && "ml-80"
        )}
      >
        <ReactFlowProvider>
          <ReactFlow
            nodes={nodes}
            edges={edges}
            onNodesChange={onNodesChange}
            onEdgesChange={onEdgesChange}
            onConnect={onConnect}
            onDragOver={onDragOver}
            onDrop={onDrop}
            onInit={setReactFlowInstance}
            nodeTypes={nodeTypes}
            edgeTypes={edgeTypes}
            fitView
            className="react-flow-wrapper"
          >
            <Background />
            <Controls className="!left-auto !right-8" />
            <MiniMap
              nodeColor={(node) => {
                const action = availableActions.find(
                  (a) => a.type === node.data.type
                );
                return action ? "hsl(var(--primary) / 0.5)" : "hsl(var(--muted))"
              }}
              maskColor="hsl(var(--background))"
              className="!bottom-8 !left-auto !right-8 !bg-card/80 !border !border-border rounded-lg backdrop-blur"
              style={{
                backgroundColor: "hsl(var(--card))",
                borderRadius: "var(--radius)",
              }}
            />
            <Panel position="top-right" className="flex gap-2">
              <Button
                variant="outline"
                size="icon"
                onClick={undo}
                disabled={historyIndex === 0}
              >
                <Undo className="h-4 w-4" />
              </Button>
              <Button
                variant="outline"
                size="icon"
                onClick={redo}
                disabled={historyIndex === history.length - 1}
              >
                <Redo className="h-4 w-4" />
              </Button>
            </Panel>
          </ReactFlow>
        </ReactFlowProvider>
      </div>
    </div>
  );
} 