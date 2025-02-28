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
import { type ActionConfig } from "~/lib/experiments/plugin-actions";

interface ActionConfigDialogProps {
  open: boolean;
  onOpenChange: (open: boolean) => void;
  type: string;
  parameters: Record<string, any>;
  onSubmit: (parameters: Record<string, any>) => void;
  actionConfig: ActionConfig;
}

export function ActionConfigDialog({
  open,
  onOpenChange,
  type,
  parameters,
  onSubmit,
  actionConfig,
}: ActionConfigDialogProps) {
  // Create a dynamic schema based on the action's parameters
  const createDynamicSchema = () => {
    if (!actionConfig) return z.object({});

    const schemaFields: Record<string, z.ZodType<any>> = {};

    for (const [key, prop] of Object.entries(actionConfig.defaultParameters)) {
      switch (typeof prop) {
        case "string":
          schemaFields[key] = z.string();
          break;
        case "number":
          schemaFields[key] = z.number();
          break;
        case "boolean":
          schemaFields[key] = z.boolean();
          break;
        case "object":
          if (Array.isArray(prop)) {
            schemaFields[key] = z.array(z.any());
          } else {
            schemaFields[key] = z.record(z.any());
          }
          break;
        default:
          schemaFields[key] = z.any();
      }
    }

    return z.object(schemaFields);
  };

  const schema = createDynamicSchema();
  const form = useForm({
    resolver: zodResolver(schema),
    defaultValues: parameters,
  });

  function handleSubmit(data: Record<string, any>) {
    onSubmit(data);
    onOpenChange(false);
  }

  function renderField(key: string, value: any) {
    const fieldType = typeof value;

    switch (fieldType) {
      case "string":
        if (value.length > 50) {
          return (
            <FormField
              key={key}
              control={form.control}
              name={key}
              render={({ field }) => (
                <FormItem>
                  <FormLabel>{key}</FormLabel>
                  <FormControl>
                    <Textarea {...field} />
                  </FormControl>
                  <FormMessage />
                </FormItem>
              )}
            />
          );
        }
        return (
          <FormField
            key={key}
            control={form.control}
            name={key}
            render={({ field }) => (
              <FormItem>
                <FormLabel>{key}</FormLabel>
                <FormControl>
                  <Input {...field} />
                </FormControl>
                <FormMessage />
              </FormItem>
            )}
          />
        );

      case "number":
        return (
          <FormField
            key={key}
            control={form.control}
            name={key}
            render={({ field }) => (
              <FormItem>
                <FormLabel>{key}</FormLabel>
                <FormControl>
                  <Input
                    type="number"
                    {...field}
                    onChange={(e) => field.onChange(parseFloat(e.target.value))}
                  />
                </FormControl>
                <FormMessage />
              </FormItem>
            )}
          />
        );

      case "boolean":
        return (
          <FormField
            key={key}
            control={form.control}
            name={key}
            render={({ field }) => (
              <FormItem>
                <div className="flex items-center gap-2">
                  <FormControl>
                    <Switch
                      checked={field.value}
                      onCheckedChange={field.onChange}
                    />
                  </FormControl>
                  <FormLabel>{key}</FormLabel>
                </div>
                <FormMessage />
              </FormItem>
            )}
          />
        );

      case "object":
        if (Array.isArray(value)) {
          // TODO: Add array field handling
          return null;
        }
        // TODO: Add object field handling
        return null;

      default:
        return null;
    }
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
            {Object.entries(actionConfig.defaultParameters).map(([key, value]) =>
              renderField(key, value)
            )}
            <Button type="submit">Save Changes</Button>
          </form>
        </Form>
      </DialogContent>
    </Dialog>
  );
} 