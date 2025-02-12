"use client";

import { useForm } from "react-hook-form";
import { zodResolver } from "@hookform/resolvers/zod";
import { z } from "zod";
import {
  Dialog,
  DialogContent,
  DialogDescription,
  DialogHeader,
  DialogTitle,
} from "~/components/ui/dialog";
import {
  Form,
  FormControl,
  FormDescription,
  FormField,
  FormItem,
  FormLabel,
  FormMessage,
} from "~/components/ui/form";
import { Input } from "~/components/ui/input";
import { Button } from "~/components/ui/button";
import { Textarea } from "~/components/ui/textarea";
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from "~/components/ui/select";
import { Switch } from "~/components/ui/switch";
import { AVAILABLE_ACTIONS } from "~/lib/experiments/actions";
import { type ActionType } from "~/lib/experiments/types";

// Define parameter schemas for each action type
const parameterSchemas = {
  move: z.object({
    position: z.object({
      x: z.number(),
      y: z.number(),
      z: z.number(),
    }),
    speed: z.number().min(0).max(1),
    easing: z.enum(["linear", "easeIn", "easeOut", "easeInOut"]),
  }),
  speak: z.object({
    text: z.string().min(1),
    speed: z.number().min(0.5).max(2),
    pitch: z.number().min(0.5).max(2),
    volume: z.number().min(0).max(1),
  }),
  wait: z.object({
    duration: z.number().min(0),
    showCountdown: z.boolean(),
  }),
  input: z.object({
    type: z.enum(["button", "text", "number", "choice"]),
    prompt: z.string().optional(),
    options: z.array(z.string()).optional(),
    timeout: z.number().nullable(),
  }),
  gesture: z.object({
    name: z.string().min(1),
    speed: z.number().min(0).max(1),
    intensity: z.number().min(0).max(1),
  }),
  record: z.object({
    type: z.enum(["start", "stop"]),
    streams: z.array(z.enum(["video", "audio", "sensors"])),
  }),
  condition: z.object({
    condition: z.string().min(1),
    trueActions: z.array(z.any()),
    falseActions: z.array(z.any()).optional(),
  }),
  loop: z.object({
    count: z.number().min(1),
    actions: z.array(z.any()),
  }),
} satisfies Record<ActionType, z.ZodType<any>>;

interface ActionConfigDialogProps {
  open: boolean;
  onOpenChange: (open: boolean) => void;
  type: ActionType;
  parameters: Record<string, any>;
  onSubmit: (parameters: Record<string, any>) => void;
}

export function ActionConfigDialog({
  open,
  onOpenChange,
  type,
  parameters,
  onSubmit,
}: ActionConfigDialogProps) {
  const actionConfig = AVAILABLE_ACTIONS.find((a) => a.type === type);
  if (!actionConfig) return null;

  const schema = parameterSchemas[type];
  const form = useForm({
    resolver: zodResolver(schema),
    defaultValues: parameters,
  });

  function handleSubmit(data: Record<string, any>) {
    onSubmit(data);
    onOpenChange(false);
  }

  return (
    <Dialog open={open} onOpenChange={onOpenChange}>
      <DialogContent>
        <DialogHeader>
          <DialogTitle>Configure {actionConfig.title}</DialogTitle>
          <DialogDescription>{actionConfig.description}</DialogDescription>
        </DialogHeader>

        <Form {...form}>
          <form onSubmit={form.handleSubmit(handleSubmit)} className="space-y-4">
            {type === "move" && (
              <>
                <div className="grid gap-4 sm:grid-cols-3">
                  <FormField
                    control={form.control}
                    name="position.x"
                    render={({ field }) => (
                      <FormItem>
                        <FormLabel>X Position</FormLabel>
                        <FormControl>
                          <Input
                            type="number"
                            step="0.1"
                            {...field}
                            onChange={(e) =>
                              field.onChange(parseFloat(e.target.value))
                            }
                          />
                        </FormControl>
                        <FormMessage />
                      </FormItem>
                    )}
                  />
                  <FormField
                    control={form.control}
                    name="position.y"
                    render={({ field }) => (
                      <FormItem>
                        <FormLabel>Y Position</FormLabel>
                        <FormControl>
                          <Input
                            type="number"
                            step="0.1"
                            {...field}
                            onChange={(e) =>
                              field.onChange(parseFloat(e.target.value))
                            }
                          />
                        </FormControl>
                        <FormMessage />
                      </FormItem>
                    )}
                  />
                  <FormField
                    control={form.control}
                    name="position.z"
                    render={({ field }) => (
                      <FormItem>
                        <FormLabel>Z Position</FormLabel>
                        <FormControl>
                          <Input
                            type="number"
                            step="0.1"
                            {...field}
                            onChange={(e) =>
                              field.onChange(parseFloat(e.target.value))
                            }
                          />
                        </FormControl>
                        <FormMessage />
                      </FormItem>
                    )}
                  />
                </div>
                <FormField
                  control={form.control}
                  name="speed"
                  render={({ field }) => (
                    <FormItem>
                      <FormLabel>Speed</FormLabel>
                      <FormControl>
                        <Input
                          type="number"
                          step="0.1"
                          min="0"
                          max="1"
                          {...field}
                          onChange={(e) =>
                            field.onChange(parseFloat(e.target.value))
                          }
                        />
                      </FormControl>
                      <FormDescription>
                        Movement speed (0-1)
                      </FormDescription>
                      <FormMessage />
                    </FormItem>
                  )}
                />
                <FormField
                  control={form.control}
                  name="easing"
                  render={({ field }) => (
                    <FormItem>
                      <FormLabel>Easing</FormLabel>
                      <Select
                        onValueChange={field.onChange}
                        defaultValue={field.value}
                      >
                        <FormControl>
                          <SelectTrigger>
                            <SelectValue placeholder="Select easing type" />
                          </SelectTrigger>
                        </FormControl>
                        <SelectContent>
                          <SelectItem value="linear">Linear</SelectItem>
                          <SelectItem value="easeIn">Ease In</SelectItem>
                          <SelectItem value="easeOut">Ease Out</SelectItem>
                          <SelectItem value="easeInOut">Ease In Out</SelectItem>
                        </SelectContent>
                      </Select>
                      <FormDescription>
                        Movement easing function
                      </FormDescription>
                      <FormMessage />
                    </FormItem>
                  )}
                />
              </>
            )}

            {type === "speak" && (
              <>
                <FormField
                  control={form.control}
                  name="text"
                  render={({ field }) => (
                    <FormItem>
                      <FormLabel>Text</FormLabel>
                      <FormControl>
                        <Textarea
                          placeholder="Enter text to speak"
                          className="resize-none"
                          {...field}
                        />
                      </FormControl>
                      <FormMessage />
                    </FormItem>
                  )}
                />
                <div className="grid gap-4 sm:grid-cols-3">
                  <FormField
                    control={form.control}
                    name="speed"
                    render={({ field }) => (
                      <FormItem>
                        <FormLabel>Speed</FormLabel>
                        <FormControl>
                          <Input
                            type="number"
                            step="0.1"
                            min="0.5"
                            max="2"
                            {...field}
                            onChange={(e) =>
                              field.onChange(parseFloat(e.target.value))
                            }
                          />
                        </FormControl>
                        <FormMessage />
                      </FormItem>
                    )}
                  />
                  <FormField
                    control={form.control}
                    name="pitch"
                    render={({ field }) => (
                      <FormItem>
                        <FormLabel>Pitch</FormLabel>
                        <FormControl>
                          <Input
                            type="number"
                            step="0.1"
                            min="0.5"
                            max="2"
                            {...field}
                            onChange={(e) =>
                              field.onChange(parseFloat(e.target.value))
                            }
                          />
                        </FormControl>
                        <FormMessage />
                      </FormItem>
                    )}
                  />
                  <FormField
                    control={form.control}
                    name="volume"
                    render={({ field }) => (
                      <FormItem>
                        <FormLabel>Volume</FormLabel>
                        <FormControl>
                          <Input
                            type="number"
                            step="0.1"
                            min="0"
                            max="1"
                            {...field}
                            onChange={(e) =>
                              field.onChange(parseFloat(e.target.value))
                            }
                          />
                        </FormControl>
                        <FormMessage />
                      </FormItem>
                    )}
                  />
                </div>
              </>
            )}

            {type === "wait" && (
              <>
                <FormField
                  control={form.control}
                  name="duration"
                  render={({ field }) => (
                    <FormItem>
                      <FormLabel>Duration (ms)</FormLabel>
                      <FormControl>
                        <Input
                          type="number"
                          min="0"
                          step="100"
                          {...field}
                          onChange={(e) =>
                            field.onChange(parseFloat(e.target.value))
                          }
                        />
                      </FormControl>
                      <FormDescription>
                        Wait duration in milliseconds
                      </FormDescription>
                      <FormMessage />
                    </FormItem>
                  )}
                />
                <FormField
                  control={form.control}
                  name="showCountdown"
                  render={({ field }) => (
                    <FormItem className="flex flex-row items-center justify-between rounded-lg border p-4">
                      <div className="space-y-0.5">
                        <FormLabel className="text-base">
                          Show Countdown
                        </FormLabel>
                        <FormDescription>
                          Display a countdown timer during the wait
                        </FormDescription>
                      </div>
                      <FormControl>
                        <Switch
                          checked={field.value}
                          onCheckedChange={field.onChange}
                        />
                      </FormControl>
                    </FormItem>
                  )}
                />
              </>
            )}

            {/* Add more action type configurations here */}

            <div className="flex justify-end gap-4">
              <Button
                type="button"
                variant="outline"
                onClick={() => onOpenChange(false)}
              >
                Cancel
              </Button>
              <Button type="submit">Save Changes</Button>
            </div>
          </form>
        </Form>
      </DialogContent>
    </Dialog>
  );
} 