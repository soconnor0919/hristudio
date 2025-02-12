"use client";

import { BaseEdge, EdgeProps, getBezierPath } from "reactflow";
import { motion } from "framer-motion";

export function FlowEdge({
  id,
  sourceX,
  sourceY,
  targetX,
  targetY,
  sourcePosition,
  targetPosition,
  style = {},
  markerEnd,
}: EdgeProps) {
  const [edgePath] = getBezierPath({
    sourceX,
    sourceY,
    sourcePosition,
    targetX,
    targetY,
    targetPosition,
  });

  return (
    <>
      <BaseEdge path={edgePath} markerEnd={markerEnd} style={style} />
      <motion.path
        id={id}
        style={{
          ...style,
          strokeWidth: 3,
          fill: "none",
          stroke: "hsl(var(--primary))",
          strokeDasharray: "5,5",
          opacity: 0.5,
        }}
        d={edgePath}
        className="react-flow__edge-path"
        animate={{
          strokeDashoffset: [0, -10],
        }}
        transition={{
          duration: 1,
          repeat: Infinity,
          ease: "linear",
        }}
      />
      <motion.path
        style={{
          strokeWidth: 15,
          fill: "none",
          stroke: "hsl(var(--primary))",
          opacity: 0,
          cursor: "pointer",
        }}
        d={edgePath}
        className="react-flow__edge-interaction"
        onMouseEnter={(event) => {
          const path = event.currentTarget.previousSibling as SVGPathElement;
          if (path) {
            path.style.opacity = "1";
            path.style.strokeWidth = "4";
          }
        }}
        onMouseLeave={(event) => {
          const path = event.currentTarget.previousSibling as SVGPathElement;
          if (path) {
            path.style.opacity = "0.5";
            path.style.strokeWidth = "3";
          }
        }}
      />
    </>
  );
} 