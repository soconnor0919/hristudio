"use client";

import { useState } from "react";
import { Input } from "~/components/ui/input";
import { Label } from "~/components/ui/label";
import { Button } from "~/components/ui/button";
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from "~/components/ui/select";
import { Badge } from "~/components/ui/badge";
import { Trash2, Plus, GripVertical } from "lucide-react";
import type { FormField, FormFieldType } from "~/lib/types/forms";
import { FORM_FIELD_TYPES } from "~/lib/types/forms";

interface FormBuilderProps {
  fields: FormField[];
  onFieldsChange: (fields: FormField[]) => void;
  disabled?: boolean;
}

export function FormBuilder({ fields, onFieldsChange, disabled = false }: FormBuilderProps) {
  const addField = (type: string) => {
    const newField: FormField = {
      id: crypto.randomUUID(),
      type: type as FormFieldType,
      label: `New ${FORM_FIELD_TYPES.find((f) => f.value === type)?.label || "Field"}`,
      required: false,
      options: type === "multiple_choice" ? ["Option 1", "Option 2"] : undefined,
    };
    onFieldsChange([...fields, newField]);
  };

  const removeField = (id: string) => {
    onFieldsChange(fields.filter((f) => f.id !== id));
  };

  const updateField = (id: string, updates: Partial<FormField>) => {
    onFieldsChange(fields.map((f) => (f.id === id ? { ...f, ...updates } : f)));
  };

  return (
    <div className="space-y-4">
      {fields.length === 0 ? (
        <div className="flex flex-col items-center justify-center py-8 text-center text-muted-foreground">
          <p>No fields added yet</p>
          <p className="text-sm">Use the dropdown below to add fields</p>
        </div>
      ) : (
        fields.map((field) => (
          <div
            key={field.id}
            className="flex items-start gap-3 rounded-lg border p-4"
          >
            <div className="flex cursor-grab items-center text-muted-foreground">
              <GripVertical className="h-5 w-5" />
            </div>
            <div className="flex-1 space-y-3">
              <div className="flex items-center gap-3">
                <Badge variant="outline" className="text-xs">
                  {FORM_FIELD_TYPES.find((f) => f.value === field.type)?.icon}{" "}
                  {FORM_FIELD_TYPES.find((f) => f.value === field.type)?.label}
                </Badge>
                <Input
                  value={field.label}
                  onChange={(e) => updateField(field.id, { label: e.target.value })}
                  placeholder="Field label"
                  className="flex-1"
                  disabled={disabled}
                />
                <label className="flex items-center gap-2 text-sm">
                  <input
                    type="checkbox"
                    checked={field.required}
                    onChange={(e) => updateField(field.id, { required: e.target.checked })}
                    className="rounded border-gray-300"
                    disabled={disabled}
                  />
                  Required
                </label>
              </div>
              {field.type === "multiple_choice" && (
                <div className="space-y-2">
                  <Label className="text-xs">Options</Label>
                  {field.options?.map((opt, i) => (
                    <div key={i} className="flex items-center gap-2">
                      <Input
                        value={opt}
                        onChange={(e) => {
                          const newOptions = [...(field.options || [])];
                          newOptions[i] = e.target.value;
                          updateField(field.id, { options: newOptions });
                        }}
                        placeholder={`Option ${i + 1}`}
                        className="flex-1"
                        disabled={disabled}
                      />
                      <Button
                        type="button"
                        variant="ghost"
                        size="icon"
                        onClick={() => {
                          const newOptions = field.options?.filter((_, idx) => idx !== i);
                          updateField(field.id, { options: newOptions });
                        }}
                        disabled={disabled}
                      >
                        <Trash2 className="h-4 w-4" />
                      </Button>
                    </div>
                  ))}
                  <Button
                    type="button"
                    variant="outline"
                    size="sm"
                    onClick={() => {
                      const newOptions = [
                        ...(field.options || []),
                        `Option ${(field.options?.length || 0) + 1}`,
                      ];
                      updateField(field.id, { options: newOptions });
                    }}
                    disabled={disabled}
                  >
                    <Plus className="mr-1 h-4 w-4" />
                    Add Option
                  </Button>
                </div>
              )}
              {field.type === "rating" && (
                <div className="flex items-center gap-2 text-sm text-muted-foreground">
                  <span>Scale:</span>
                  <Select
                    value={String(field.settings?.scale || 5)}
                    onValueChange={(val) =>
                      updateField(field.id, { settings: { scale: parseInt(val) } })
                    }
                    disabled={disabled}
                  >
                    <SelectTrigger className="w-[100px]">
                      <SelectValue />
                    </SelectTrigger>
                    <SelectContent>
                      <SelectItem value="5">1-5</SelectItem>
                      <SelectItem value="7">1-7</SelectItem>
                      <SelectItem value="10">1-10</SelectItem>
                    </SelectContent>
                  </Select>
                </div>
              )}
            </div>
            <Button
              type="button"
              variant="ghost"
              size="icon"
              onClick={() => removeField(field.id)}
              disabled={disabled}
            >
              <Trash2 className="h-4 w-4 text-destructive" />
            </Button>
          </div>
        ))
      )}
    </div>
  );
}
