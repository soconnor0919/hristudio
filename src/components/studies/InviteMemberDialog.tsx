"use client";

import { zodResolver } from "@hookform/resolvers/zod";
import { useState } from "react";
import { useForm } from "react-hook-form";
import { z } from "zod";
import { Button } from "~/components/ui/button";
import {
  Dialog,
  DialogContent,
  DialogDescription,
  DialogFooter,
  DialogHeader,
  DialogTitle,
  DialogTrigger,
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
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from "~/components/ui/select";

import { Mail, Plus, UserPlus } from "lucide-react";
import { Badge } from "~/components/ui/badge";
import { useStudyManagement } from "~/hooks/useStudyManagement";

const inviteSchema = z.object({
  email: z.string().email("Please enter a valid email address"),
  role: z.enum(["researcher", "wizard", "observer"], {
    message: "Please select a role",
  }),
});

type InviteFormData = z.infer<typeof inviteSchema>;

interface InviteMemberDialogProps {
  studyId: string;
  children?: React.ReactNode;
}

const roleDescriptions = {
  researcher: {
    label: "Researcher",
    description: "Can manage experiments, view all data, and invite members",
    icon: "🔬",
  },
  wizard: {
    label: "Wizard",
    description: "Can control trials and execute experiments",
    icon: "🎭",
  },
  observer: {
    label: "Observer",
    description: "Read-only access to view trials and data",
    icon: "👁️",
  },
};

export function InviteMemberDialog({
  studyId,
  children,
}: InviteMemberDialogProps) {
  const [open, setOpen] = useState(false);

  const form = useForm<InviteFormData>({
    resolver: zodResolver(inviteSchema),
    defaultValues: {
      email: "",
      role: undefined,
    },
  });

  const { addStudyMember } = useStudyManagement();

  const handleAddMember = async (data: InviteFormData) => {
    try {
      await addStudyMember(studyId, data.email, data.role);
      form.reset();
      setOpen(false);
    } catch {
      // Error handling is done in the hook
    }
  };

  const onSubmit = (data: InviteFormData) => {
    void handleAddMember(data);
  };

  return (
    <Dialog open={open} onOpenChange={setOpen}>
      <DialogTrigger asChild>
        {children ?? (
          <Button variant="outline" size="sm">
            <Plus className="mr-2 h-4 w-4" />
            Invite
          </Button>
        )}
      </DialogTrigger>
      <DialogContent className="sm:max-w-md">
        <DialogHeader>
          <DialogTitle className="flex items-center space-x-2">
            <UserPlus className="h-5 w-5" />
            <span>Invite Team Member</span>
          </DialogTitle>
          <DialogDescription>
            Add a team member to this research study. They must have an existing
            account with the email address you provide.
          </DialogDescription>
        </DialogHeader>

        <Form {...form}>
          <form onSubmit={form.handleSubmit(onSubmit)} className="space-y-6">
            <FormField
              control={form.control}
              name="email"
              render={({ field }) => (
                <FormItem>
                  <FormLabel>Email Address</FormLabel>
                  <FormControl>
                    <div className="relative">
                      <Mail className="absolute top-3 left-3 h-4 w-4 text-slate-400" />
                      <Input
                        {...field}
                        placeholder="colleague@university.edu"
                        className="pl-10"
                      />
                    </div>
                  </FormControl>
                  <FormDescription>
                    Enter the email address of the person you want to add (they
                    must have an account)
                  </FormDescription>
                  <FormMessage />
                </FormItem>
              )}
            />

            <FormField
              control={form.control}
              name="role"
              render={({ field }) => (
                <FormItem>
                  <FormLabel>Role</FormLabel>
                  <Select
                    onValueChange={field.onChange}
                    defaultValue={field.value}
                  >
                    <FormControl>
                      <SelectTrigger>
                        <SelectValue placeholder="Select a role for this member" />
                      </SelectTrigger>
                    </FormControl>
                    <SelectContent>
                      {Object.entries(roleDescriptions).map(
                        ([value, config]) => (
                          <SelectItem key={value} value={value}>
                            <div className="flex items-center space-x-2">
                              <span>{config.icon}</span>
                              <span>{config.label}</span>
                            </div>
                          </SelectItem>
                        ),
                      )}
                    </SelectContent>
                  </Select>

                  {field.value && (
                    <div className="mt-2 rounded-lg bg-slate-50 p-3">
                      <div className="mb-1 flex items-center space-x-2">
                        <Badge variant="secondary" className="text-xs">
                          {roleDescriptions[field.value].icon}{" "}
                          {roleDescriptions[field.value].label}
                        </Badge>
                      </div>
                      <p className="text-xs text-slate-600">
                        {roleDescriptions[field.value].description}
                      </p>
                    </div>
                  )}
                  <FormMessage />
                </FormItem>
              )}
            />

            <DialogFooter className="gap-2">
              <Button
                type="button"
                variant="outline"
                onClick={() => setOpen(false)}
              >
                Cancel
              </Button>
              <Button type="submit">Add Member</Button>
            </DialogFooter>
          </form>
        </Form>
      </DialogContent>
    </Dialog>
  );
}
