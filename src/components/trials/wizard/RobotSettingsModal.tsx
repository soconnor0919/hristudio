import { useState } from "react";
import { Dialog, DialogContent, DialogDescription, DialogFooter, DialogHeader, DialogTitle } from "~/components/ui/dialog";
import { Button } from "~/components/ui/button";
import { Input } from "~/components/ui/input";
import { Label } from "~/components/ui/label";
import { Switch } from "~/components/ui/switch";
import { Select, SelectContent, SelectItem, SelectTrigger, SelectValue } from "~/components/ui/select";
import { Separator } from "~/components/ui/separator";
import { Loader2, Settings2 } from "lucide-react";
import { api } from "~/trpc/react";
import { toast } from "sonner";

interface RobotSettingsModalProps {
    open: boolean;
    onOpenChange: (open: boolean) => void;
    studyId: string;
    pluginId: string;
    settingsSchema: SettingsSchema | null;
}

interface SettingsSchema {
    type: "object";
    title?: string;
    description?: string;
    properties: Record<string, PropertySchema>;
}

interface PropertySchema {
    type: "object" | "string" | "number" | "integer" | "boolean";
    title?: string;
    description?: string;
    properties?: Record<string, PropertySchema>;
    enum?: string[];
    enumNames?: string[];
    minimum?: number;
    maximum?: number;
    default?: unknown;
    pattern?: string;
}

export function RobotSettingsModal({
    open,
    onOpenChange,
    studyId,
    pluginId,
    settingsSchema,
}: RobotSettingsModalProps) {
    const [settings, setSettings] = useState<Record<string, unknown>>({});
    const [isSaving, setIsSaving] = useState(false);

    // Fetch current settings
    const { data: currentSettings, isLoading } = api.studies.getPluginConfiguration.useQuery(
        { studyId, pluginId },
        { enabled: open }
    );

    // Update settings mutation
    const updateSettings = api.studies.updatePluginConfiguration.useMutation({
        onSuccess: () => {
            toast.success("Robot settings updated successfully");
            onOpenChange(false);
        },
        onError: (error: { message: string }) => {
            toast.error(`Failed to update settings: ${error.message}`);
        },
    });

    // Initialize settings from current configuration
    // eslint-disable-next-line react-hooks/exhaustive-deps
    useState(() => {
        if (currentSettings) {
            setSettings(currentSettings as Record<string, unknown>);
        }
    });

    const handleSave = async () => {
        setIsSaving(true);
        try {
            await updateSettings.mutateAsync({
                studyId,
                pluginId,
                configuration: settings,
            });
        } finally {
            setIsSaving(false);
        }
    };

    const renderField = (key: string, schema: PropertySchema, parentPath: string = "") => {
        const fullPath = parentPath ? `${parentPath}.${key}` : key;
        const value = getNestedValue(settings, fullPath);
        const defaultValue = schema.default;

        const updateValue = (newValue: unknown) => {
            setSettings((prev) => setNestedValue({ ...prev }, fullPath, newValue));
        };

        // Object type - render nested fields
        if (schema.type === "object" && schema.properties) {
            return (
                <div key={fullPath} className="space-y-4">
                    <div className="space-y-1">
                        <h4 className="text-sm font-semibold">{schema.title || key}</h4>
                        {schema.description && (
                            <p className="text-xs text-muted-foreground">{schema.description}</p>
                        )}
                    </div>
                    <div className="ml-4 space-y-3">
                        {Object.entries(schema.properties).map(([subKey, subSchema]) =>
                            renderField(subKey, subSchema, fullPath)
                        )}
                    </div>
                </div>
            );
        }

        // Boolean type - render switch
        if (schema.type === "boolean") {
            return (
                <div key={fullPath} className="flex items-center justify-between space-x-2">
                    <div className="space-y-0.5 flex-1">
                        <Label htmlFor={fullPath}>{schema.title || key}</Label>
                        {schema.description && (
                            <p className="text-xs text-muted-foreground">{schema.description}</p>
                        )}
                    </div>
                    <Switch
                        id={fullPath}
                        checked={(value ?? defaultValue) as boolean}
                        onCheckedChange={updateValue}
                    />
                </div>
            );
        }

        // Enum type - render select
        if (schema.enum) {
            return (
                <div key={fullPath} className="space-y-2">
                    <Label htmlFor={fullPath}>{schema.title || key}</Label>
                    {schema.description && (
                        <p className="text-xs text-muted-foreground">{schema.description}</p>
                    )}
                    <Select
                        value={(value ?? defaultValue) as string}
                        onValueChange={updateValue}
                    >
                        <SelectTrigger id={fullPath}>
                            <SelectValue placeholder="Select an option" />
                        </SelectTrigger>
                        <SelectContent>
                            {schema.enum.map((option, idx) => (
                                <SelectItem key={option} value={option}>
                                    {schema.enumNames?.[idx] || option}
                                </SelectItem>
                            ))}
                        </SelectContent>
                    </Select>
                </div>
            );
        }

        // Number/Integer type - render number input
        if (schema.type === "number" || schema.type === "integer") {
            return (
                <div key={fullPath} className="space-y-2">
                    <Label htmlFor={fullPath}>{schema.title || key}</Label>
                    {schema.description && (
                        <p className="text-xs text-muted-foreground">{schema.description}</p>
                    )}
                    <Input
                        id={fullPath}
                        type="number"
                        min={schema.minimum}
                        max={schema.maximum}
                        step={schema.type === "integer" ? 1 : 0.1}
                        value={(value ?? defaultValue) as number}
                        onChange={(e) => {
                            const newValue = schema.type === "integer"
                                ? parseInt(e.target.value, 10)
                                : parseFloat(e.target.value);
                            updateValue(isNaN(newValue) ? defaultValue : newValue);
                        }}
                    />
                </div>
            );
        }

        // String type - render text input
        return (
            <div key={fullPath} className="space-y-2">
                <Label htmlFor={fullPath}>{schema.title || key}</Label>
                {schema.description && (
                    <p className="text-xs text-muted-foreground">{schema.description}</p>
                )}
                <Input
                    id={fullPath}
                    type="text"
                    pattern={schema.pattern}
                    value={(value ?? defaultValue) as string}
                    onChange={(e) => updateValue(e.target.value)}
                />
            </div>
        );
    };

    if (!settingsSchema) {
        return null;
    }

    return (
        <Dialog open={open} onOpenChange={onOpenChange}>
            <DialogContent className="max-w-2xl max-h-[80vh] overflow-y-auto">
                <DialogHeader>
                    <DialogTitle className="flex items-center gap-2">
                        <Settings2 className="h-5 w-5" />
                        {settingsSchema.title || "Robot Settings"}
                    </DialogTitle>
                    {settingsSchema.description && (
                        <DialogDescription>{settingsSchema.description}</DialogDescription>
                    )}
                </DialogHeader>

                {isLoading ? (
                    <div className="flex items-center justify-center py-8">
                        <Loader2 className="h-6 w-6 animate-spin text-muted-foreground" />
                    </div>
                ) : (
                    <div className="space-y-6 py-4">
                        {Object.entries(settingsSchema.properties).map(([key, schema], idx) => (
                            <div key={key}>
                                {renderField(key, schema)}
                                {idx < Object.keys(settingsSchema.properties).length - 1 && (
                                    <Separator className="mt-6" />
                                )}
                            </div>
                        ))}
                    </div>
                )}

                <DialogFooter>
                    <Button variant="outline" onClick={() => onOpenChange(false)} disabled={isSaving}>
                        Cancel
                    </Button>
                    <Button onClick={handleSave} disabled={isSaving || isLoading}>
                        {isSaving && <Loader2 className="mr-2 h-4 w-4 animate-spin" />}
                        Save Settings
                    </Button>
                </DialogFooter>
            </DialogContent>
        </Dialog>
    );
}

// Helper functions for nested object access
function getNestedValue(obj: Record<string, unknown>, path: string): unknown {
    return path.split(".").reduce((current, key) => {
        return current && typeof current === "object" ? (current as Record<string, unknown>)[key] : undefined;
    }, obj as unknown);
}

function setNestedValue(obj: Record<string, unknown>, path: string, value: unknown): Record<string, unknown> {
    const keys = path.split(".");
    const lastKey = keys.pop()!;
    const target = keys.reduce((current, key) => {
        if (!current[key] || typeof current[key] !== "object") {
            current[key] = {};
        }
        return current[key] as Record<string, unknown>;
    }, obj);
    target[lastKey] = value;
    return obj;
}
