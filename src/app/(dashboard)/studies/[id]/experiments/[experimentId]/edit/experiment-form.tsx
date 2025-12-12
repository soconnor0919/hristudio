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
import { toast } from "sonner";
import { api } from "~/trpc/react";
import { useRouter } from "next/navigation";
import { type Experiment } from "~/lib/experiments/types";

const formSchema = z.object({
    name: z.string().min(2, {
        message: "Name must be at least 2 characters.",
    }),
    description: z.string().optional(),
    status: z.enum([
        "draft",
        "ready",
        "data_collection",
        "analysis",
        "completed",
        "archived",
    ]),
});

interface ExperimentFormProps {
    experiment: Experiment;
}

export function ExperimentForm({ experiment }: ExperimentFormProps) {
    const router = useRouter();
    const updateExperiment = api.experiments.update.useMutation({
        onSuccess: () => {
            toast.success("Experiment updated successfully");
            router.refresh();
            router.push(`/studies/${experiment.studyId}/experiments/${experiment.id}`);
        },
        onError: (error) => {
            toast.error(`Error updating experiment: ${error.message}`);
        },
    });

    const form = useForm<z.infer<typeof formSchema>>({
        resolver: zodResolver(formSchema),
        defaultValues: {
            name: experiment.name,
            description: experiment.description ?? "",
            status: experiment.status,
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

    return (
        <Form {...form}>
            <form onSubmit={form.handleSubmit(onSubmit)} className="space-y-6">
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
                                The name of your experiment.
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
                                    placeholder="Describe your experiment..."
                                    className="resize-none"
                                    {...field}
                                />
                            </FormControl>
                            <FormDescription>
                                A short description of the experiment goals.
                            </FormDescription>
                            <FormMessage />
                        </FormItem>
                    )}
                />

                <FormField
                    control={form.control}
                    name="status"
                    render={({ field }) => (
                        <FormItem>
                            <FormLabel>Status</FormLabel>
                            <Select onValueChange={field.onChange} defaultValue={field.value}>
                                <FormControl>
                                    <SelectTrigger>
                                        <SelectValue placeholder="Select a status" />
                                    </SelectTrigger>
                                </FormControl>
                                <SelectContent>
                                    <SelectItem value="draft">Draft</SelectItem>
                                    <SelectItem value="ready">Ready</SelectItem>
                                    <SelectItem value="data_collection">Data Collection</SelectItem>
                                    <SelectItem value="analysis">Analysis</SelectItem>
                                    <SelectItem value="completed">Completed</SelectItem>
                                    <SelectItem value="archived">Archived</SelectItem>
                                </SelectContent>
                            </Select>
                            <FormDescription>
                                The current status of the experiment.
                            </FormDescription>
                            <FormMessage />
                        </FormItem>
                    )}
                />

                <div className="flex gap-4">
                    <Button type="submit" disabled={updateExperiment.isPending}>
                        {updateExperiment.isPending ? "Saving..." : "Save Changes"}
                    </Button>
                    <Button
                        type="button"
                        variant="outline"
                        onClick={() =>
                            router.push(
                                `/studies/${experiment.studyId}/experiments/${experiment.id}`,
                            )
                        }
                    >
                        Cancel
                    </Button>
                </div>
            </form>
        </Form>
    );
}
