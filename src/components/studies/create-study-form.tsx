"use client";

import { useRouter } from "next/navigation";
import { zodResolver } from "@hookform/resolvers/zod";
import { useForm } from "react-hook-form";
import { z } from "zod";
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
import { useToast } from "~/hooks/use-toast";
import { api } from "~/trpc/react";
import { useSession } from "next-auth/react";

const createStudySchema = z.object({
  title: z.string().min(1, "Title is required").max(256, "Title is too long"),
  description: z.string().optional(),
});

type FormData = z.infer<typeof createStudySchema>;

export function CreateStudyForm() {
  const router = useRouter();
  const { toast } = useToast();
  const { data: session, status } = useSession();

  const form = useForm<FormData>({
    resolver: zodResolver(createStudySchema),
    defaultValues: {
      title: "",
      description: "",
    },
  });

  const createStudy = api.study.create.useMutation({
    onSuccess: (study) => {
      toast({
        title: "Study created",
        description: "Your study has been created successfully.",
      });
      router.push(`/dashboard/studies/${study.id}`);
    },
    onError: (error) => {
      toast({
        title: "Error",
        description: error.message,
        variant: "destructive",
      });
    },
  });

  const onSubmit = (data: FormData) => {
    if (status !== "authenticated") {
      toast({
        title: "Error",
        description: "You must be logged in to create a study.",
        variant: "destructive",
      });
      return;
    }
    createStudy.mutate(data);
  };

  return (
    <Form {...form}>
      <form onSubmit={form.handleSubmit(onSubmit)} className="space-y-8">
        <FormField
          control={form.control}
          name="title"
          render={({ field }) => (
            <FormItem>
              <FormLabel>Title</FormLabel>
              <FormControl>
                <Input placeholder="Enter study title" {...field} />
              </FormControl>
              <FormDescription>
                A descriptive name for your study.
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
                  placeholder="Enter study description"
                  className="resize-none"
                  {...field}
                />
              </FormControl>
              <FormDescription>
                A brief description of your study and its objectives.
              </FormDescription>
              <FormMessage />
            </FormItem>
          )}
        />

        <div className="flex justify-end gap-4">
          <Button
            type="button"
            variant="outline"
            onClick={() => router.back()}
          >
            Cancel
          </Button>
          <Button
            type="submit"
            disabled={createStudy.isLoading || status !== "authenticated"}
          >
            {createStudy.isLoading ? "Creating..." : "Create Study"}
          </Button>
        </div>
      </form>
    </Form>
  );
} 