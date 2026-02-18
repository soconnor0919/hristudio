"use client";

import { useForm } from "react-hook-form";
import { zodResolver } from "@hookform/resolvers/zod";
import * as z from "zod";
import { Button } from "~/components/ui/button";
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
import { Textarea } from "~/components/ui/textarea";
import {
    Select,
    SelectContent,
    SelectItem,
    SelectTrigger,
    SelectValue,
} from "~/components/ui/select";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "~/components/ui/card";
import { Badge } from "~/components/ui/badge";
import { toast } from "sonner";
import { api } from "~/trpc/react";
import { experimentStatusEnum } from "~/server/db/schema";
import { Save, ExternalLink } from "lucide-react";
import Link from "next/link";

const formSchema = z.object({
    name: z.string().min(2, {
        message: "Name must be at least 2 characters.",
    }),
    description: z.string().optional(),
    status: z.enum(experimentStatusEnum.enumValues),
});

interface SettingsTabProps {
    experiment: {
        id: string;
        name: string;
        description: string | null;
        status: string;
        studyId: string;
        createdAt: Date;
        updatedAt: Date;
        study: {
            id: string;
            name: string;
        };
    };
    designStats?: {
        stepCount: number;
        actionCount: number;
    };
}

export function SettingsTab({ experiment, designStats }: SettingsTabProps) {
    const utils = api.useUtils();
    const updateExperiment = api.experiments.update.useMutation({
        onSuccess: async () => {
            toast.success("Experiment settings saved successfully");
            // Invalidate experiments list to refresh data
            await utils.experiments.list.invalidate();
        },
        onError: (error) => {
            toast.error(`Error saving settings: ${error.message}`);
        },
    });

    const form = useForm<z.infer<typeof formSchema>>({
        resolver: zodResolver(formSchema),
        defaultValues: {
            name: experiment.name,
            description: experiment.description ?? "",
            status: experiment.status as z.infer<typeof formSchema>["status"],
        },
    });

    function onSubmit(values: z.infer<typeof formSchema>) {
        updateExperiment.mutate({
            id: experiment.id,
            name: values.name,
            description: values.description,
            status: values.status,
        });
    }

    const isDirty = form.formState.isDirty;

    return (
        <div className="h-full overflow-y-auto p-6">
            <div className="space-y-6">
                {/* Header */}
                <div>
                    <h2 className="text-2xl font-bold tracking-tight">Experiment Settings</h2>
                    <p className="text-muted-foreground mt-1">
                        Configure experiment metadata and status
                    </p>
                </div>

                <Form {...form}>
                    <form onSubmit={form.handleSubmit(onSubmit)} className="space-y-6">
                        <div className="grid grid-cols-1 md:grid-cols-3 gap-6">
                            {/* Left Column: Basic Information (Spans 2) */}
                            <div className="md:col-span-2 space-y-6">
                                <Card className="h-full">
                                    <CardHeader>
                                        <CardTitle>Basic Information</CardTitle>
                                        <CardDescription>
                                            The name and description help identify this experiment
                                        </CardDescription>
                                    </CardHeader>
                                    <CardContent className="space-y-4">
                                        <FormField
                                            control={form.control}
                                            name="name"
                                            render={({ field }) => (
                                                <FormItem>
                                                    <FormLabel>Name</FormLabel>
                                                    <FormControl>
                                                        <Input placeholder="Experiment name" {...field} />
                                                    </FormControl>
                                                    <FormDescription>
                                                        A clear, descriptive name for your experiment
                                                    </FormDescription>
                                                    <FormMessage />
                                                </FormItem>
                                            )}
                                        />

                                        <FormField
                                            control={form.control}
                                            name="description"
                                            render={({ field }) => (
                                                <FormItem>
                                                    <FormLabel>Description</FormLabel>
                                                    <FormControl>
                                                        <Textarea
                                                            placeholder="Describe your experiment goals, methodology, and expected outcomes..."
                                                            className="resize-none min-h-[300px]"
                                                            {...field}
                                                        />
                                                    </FormControl>
                                                    <FormDescription>
                                                        Detailed description of the experiment purpose and design
                                                    </FormDescription>
                                                    <FormMessage />
                                                </FormItem>
                                            )}
                                        />
                                    </CardContent>
                                </Card>
                            </div>

                            {/* Right Column: Status & Metadata (Spans 1) */}
                            <div className="space-y-6">
                                {/* Status Card */}
                                <Card>
                                    <CardHeader>
                                        <CardTitle>Status</CardTitle>
                                        <CardDescription>
                                            Track lifecycle stage
                                        </CardDescription>
                                    </CardHeader>
                                    <CardContent>
                                        <FormField
                                            control={form.control}
                                            name="status"
                                            render={({ field }) => (
                                                <FormItem>
                                                    <FormLabel>Current Status</FormLabel>
                                                    <Select onValueChange={field.onChange} defaultValue={field.value}>
                                                        <FormControl>
                                                            <SelectTrigger>
                                                                <SelectValue placeholder="Select a status" />
                                                            </SelectTrigger>
                                                        </FormControl>
                                                        <SelectContent>
                                                            <SelectItem value="draft">
                                                                <div className="flex items-center gap-2">
                                                                    <Badge variant="secondary">Draft</Badge>
                                                                    <span className="text-xs text-muted-foreground">WIP</span>
                                                                </div>
                                                            </SelectItem>
                                                            <SelectItem value="testing">
                                                                <div className="flex items-center gap-2">
                                                                    <Badge variant="outline">Testing</Badge>
                                                                    <span className="text-xs text-muted-foreground">Validation</span>
                                                                </div>
                                                            </SelectItem>
                                                            <SelectItem value="ready">
                                                                <div className="flex items-center gap-2">
                                                                    <Badge variant="default" className="bg-green-500">Ready</Badge>
                                                                    <span className="text-xs text-muted-foreground">Live</span>
                                                                </div>
                                                            </SelectItem>
                                                            <SelectItem value="deprecated">
                                                                <div className="flex items-center gap-2">
                                                                    <Badge variant="destructive">Deprecated</Badge>
                                                                    <span className="text-xs text-muted-foreground">Retired</span>
                                                                </div>
                                                            </SelectItem>
                                                        </SelectContent>
                                                    </Select>
                                                    <FormMessage />
                                                </FormItem>
                                            )}
                                        />
                                    </CardContent>
                                </Card>

                                {/* Metadata Card */}
                                <Card>
                                    <CardHeader>
                                        <CardTitle>Metadata</CardTitle>
                                        <CardDescription>
                                            Read-only information
                                        </CardDescription>
                                    </CardHeader>
                                    <CardContent className="space-y-4">
                                        <div className="space-y-3">
                                            <div>
                                                <p className="text-xs font-medium text-muted-foreground mb-1">Study</p>
                                                <Link
                                                    href={`/studies/${experiment.study.id}`}
                                                    className="text-sm hover:underline flex items-center gap-1 text-primary truncate"
                                                >
                                                    {experiment.study.name}
                                                    <ExternalLink className="h-3 w-3 flex-shrink-0" />
                                                </Link>
                                            </div>
                                            <div>
                                                <p className="text-xs font-medium text-muted-foreground mb-1">Experiment ID</p>
                                                <p className="text-xs font-mono bg-muted p-1 rounded select-all">{experiment.id.split('-')[0]}...</p>
                                            </div>
                                            <div className="grid grid-cols-2 gap-2">
                                                <div>
                                                    <p className="text-xs font-medium text-muted-foreground mb-1">Created</p>
                                                    <p className="text-xs">{new Date(experiment.createdAt).toLocaleDateString()}</p>
                                                </div>
                                                <div>
                                                    <p className="text-xs font-medium text-muted-foreground mb-1">Updated</p>
                                                    <p className="text-xs">{new Date(experiment.updatedAt).toLocaleDateString()}</p>
                                                </div>
                                            </div>
                                        </div>

                                        {designStats && (
                                            <div className="pt-4 border-t">
                                                <p className="text-xs font-medium text-muted-foreground mb-2">Statistics</p>
                                                <div className="flex flex-wrap gap-2">
                                                    <div className="flex items-center gap-1.5 bg-muted/50 px-2 py-1 rounded text-xs">
                                                        <span className="font-semibold">{designStats.stepCount}</span>
                                                        <span className="text-muted-foreground">Steps</span>
                                                    </div>
                                                    <div className="flex items-center gap-1.5 bg-muted/50 px-2 py-1 rounded text-xs">
                                                        <span className="font-semibold">{designStats.actionCount}</span>
                                                        <span className="text-muted-foreground">Actions</span>
                                                    </div>
                                                </div>
                                            </div>
                                        )}
                                    </CardContent>
                                </Card>
                            </div>
                        </div>

                        {/* Save Button */}
                        <div className="flex justify-end pt-4 border-t">
                            <Button
                                type="submit"
                                disabled={updateExperiment.isPending || !isDirty}
                                className="min-w-[120px]"
                            >
                                {updateExperiment.isPending ? (
                                    "Saving..."
                                ) : (
                                    <>
                                        <Save className="h-4 w-4 mr-2" />
                                        Save Changes
                                    </>
                                )}
                            </Button>
                        </div>
                    </form>
                </Form>
            </div>
        </div>
    );
}
