"use client";

import { Input } from "~/components/ui/input";
import { Label } from "~/components/ui/label";
import { Textarea } from "~/components/ui/textarea";
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from "~/components/ui/select";
import type { FormField } from "~/lib/types/forms";
import { FORM_FIELD_TYPES } from "~/lib/types/forms";

interface FormFieldRendererProps {
  field: FormField;
  value: unknown;
  onChange: (value: unknown) => void;
  mode: "preview" | "data-entry" | "participant";
  index: number;
  error?: string;
  disabled?: boolean;
}

export function FormFieldRenderer({
  field,
  value,
  onChange,
  mode,
  index,
  error,
  disabled = false,
}: FormFieldRendererProps) {
  const handleChange = (val: unknown) => {
    if (!disabled) {
      onChange(val);
    }
  };

  const commonProps = {
    disabled,
    className: error ? "border-destructive" : "",
  };

  const scale = (field.settings?.scale as number) || 5;

  switch (field.type) {
    case "text":
      return (
        <Input
          {...commonProps}
          value={String(value ?? "")}
          onChange={(e) => handleChange(e.target.value)}
          placeholder="Enter your response..."
        />
      );

    case "textarea":
      return (
        <Textarea
          {...commonProps}
          value={String(value ?? "")}
          onChange={(e) => handleChange(e.target.value)}
          placeholder="Enter your response..."
        />
      );

    case "multiple_choice": {
      const containerClass =
        mode === "participant"
          ? `mt-2 space-y-2 ${error ? "border-destructive rounded-md border p-2" : ""}`
          : "space-y-2";

      return (
        <div className={containerClass}>
          {field.options?.map((opt, i) => (
            <label key={i} className="flex cursor-pointer items-center gap-2">
              <input
                type="radio"
                name={field.id}
                value={opt}
                checked={value === opt}
                onChange={() => handleChange(opt)}
                disabled={disabled}
                className="h-4 w-4"
              />
              <span className="text-sm">{opt}</span>
            </label>
          ))}
        </div>
      );
    }

    case "checkbox":
      return (
        <div className="flex items-center gap-2">
          <input
            type="checkbox"
            checked={Boolean(value)}
            onChange={(e) => handleChange(e.target.checked)}
            disabled={disabled}
            className="h-4 w-4 rounded border-gray-300"
          />
          {mode === "participant" && (
            <Label className="cursor-pointer font-normal">Yes, I agree</Label>
          )}
        </div>
      );

    case "yes_no":
      if (mode === "data-entry") {
        return (
          <Select
            value={String(value ?? "")}
            onValueChange={(val) => handleChange(val)}
            disabled={disabled}
          >
            <SelectTrigger>
              <SelectValue placeholder="Select..." />
            </SelectTrigger>
            <SelectContent>
              <SelectItem value="yes">Yes</SelectItem>
              <SelectItem value="no">No</SelectItem>
            </SelectContent>
          </Select>
        );
      }
      return (
        <div className="flex gap-4">
          <label className="flex cursor-pointer items-center gap-2">
            <input
              type="radio"
              name={field.id}
              value="yes"
              checked={value === "yes"}
              onChange={() => handleChange("yes")}
              disabled={disabled}
              className="h-4 w-4"
            />
            <span className="text-sm">Yes</span>
          </label>
          <label className="flex cursor-pointer items-center gap-2">
            <input
              type="radio"
              name={field.id}
              value="no"
              checked={value === "no"}
              onChange={() => handleChange("no")}
              disabled={disabled}
              className="h-4 w-4"
            />
            <span className="text-sm">No</span>
          </label>
        </div>
      );

    case "rating": {
      const scale = field.settings?.scale || 5;
      if (mode === "data-entry") {
        return (
          <Select
            value={String(value ?? "")}
            onValueChange={(val) => handleChange(parseInt(val))}
            disabled={disabled}
          >
            <SelectTrigger>
              <SelectValue placeholder="Select rating..." />
            </SelectTrigger>
            <SelectContent>
              {Array.from({ length: scale }, (_, i) => (
                <SelectItem key={i} value={String(i + 1)}>
                  {i + 1}
                </SelectItem>
              ))}
            </SelectContent>
          </Select>
        );
      }
      if (mode === "participant") {
        return (
          <div className="mt-2 flex flex-wrap gap-2">
            {Array.from({ length: scale }, (_, i) => (
              <label key={i} className="cursor-pointer">
                <input
                  type="radio"
                  name={field.id}
                  value={String(i + 1)}
                  checked={value === i + 1}
                  onChange={() => handleChange(i + 1)}
                  disabled={disabled}
                  className="peer sr-only"
                />
                <span className="hover:bg-muted peer-checked:bg-primary peer-checked:text-primary-foreground flex h-10 w-10 items-center justify-center rounded-full border text-sm font-medium transition-colors">
                  {i + 1}
                </span>
              </label>
            ))}
          </div>
        );
      }
      return (
        <div className="flex gap-2">
          {Array.from({ length: scale }, (_, i) => (
            <button
              key={i}
              type="button"
              className="disabled h-8 w-8 rounded border"
              disabled
            >
              {i + 1}
            </button>
          ))}
        </div>
      );
    }

    case "date":
      return (
        <Input
          type="date"
          {...commonProps}
          value={String(value ?? "")}
          onChange={(e) => handleChange(e.target.value)}
        />
      );

    case "signature":
      return (
        <div className="space-y-2">
          <Input
            {...commonProps}
            value={String(value ?? "")}
            onChange={(e) => handleChange(e.target.value)}
            placeholder={
              mode === "participant"
                ? "Type your full name as signature"
                : "Type name as signature..."
            }
          />
          <p className="text-muted-foreground text-xs">
            By entering your name above, you confirm that the information
            provided is accurate.
          </p>
        </div>
      );

    default:
      return null;
  }
}

interface FormFieldLabelProps {
  field: FormField;
  index: number;
  showIndex?: boolean;
}

export function FormFieldLabel({
  field,
  index,
  showIndex = true,
}: FormFieldLabelProps) {
  const fieldType = FORM_FIELD_TYPES.find((f) => f.value === field.type);
  return (
    <Label>
      {showIndex && `${index + 1}. `}
      {field.label}
      {field.required && <span className="text-destructive"> *</span>}
    </Label>
  );
}
