export interface FormFieldSettings {
  scale?: number;
}

export interface FormField {
  id: string;
  type: FormFieldType;
  label: string;
  required: boolean;
  options?: string[];
  settings?: FormFieldSettings;
}

export type FormFieldType =
  | "text"
  | "textarea"
  | "multiple_choice"
  | "checkbox"
  | "rating"
  | "yes_no"
  | "date"
  | "signature";

export type FormType = "consent" | "survey" | "questionnaire";

export interface FormFieldTypeConfig {
  value: FormFieldType;
  label: string;
  icon: string;
}

export const FORM_FIELD_TYPES: FormFieldTypeConfig[] = [
  { value: "text", label: "Text (short)", icon: "📝" },
  { value: "textarea", label: "Text (long)", icon: "📄" },
  { value: "multiple_choice", label: "Multiple Choice", icon: "☑️" },
  { value: "checkbox", label: "Checkbox", icon: "✅" },
  { value: "rating", label: "Rating Scale", icon: "⭐" },
  { value: "yes_no", label: "Yes/No", icon: "✔️" },
  { value: "date", label: "Date", icon: "📅" },
  { value: "signature", label: "Signature", icon: "✍️" },
];

export function createField(type: FormFieldType): FormField {
  return {
    id: crypto.randomUUID(),
    type,
    label: `New ${FORM_FIELD_TYPES.find((f) => f.value === type)?.label || "Field"}`,
    required: false,
    options: type === "multiple_choice" ? ["Option 1", "Option 2"] : undefined,
  };
}
