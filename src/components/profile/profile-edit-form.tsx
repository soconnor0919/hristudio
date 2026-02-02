"use client";

import { zodResolver } from "@hookform/resolvers/zod";
import { useRouter } from "next/navigation";
import { useState } from "react";
import { useForm } from "react-hook-form";
import { z } from "zod";
import { Button } from "~/components/ui/button";
import { Input } from "~/components/ui/input";
import { Label } from "~/components/ui/label";
import { api } from "~/trpc/react";

const profileSchema = z.object({
  name: z.string().min(1, "Name is required").max(100, "Name is too long"),
  email: z.string().email("Invalid email address"),
});

type ProfileFormData = z.infer<typeof profileSchema>;

interface ProfileEditFormProps {
  user: {
    id: string;
    name: string | null;
    email: string | null;
    image: string | null;
  };
}

export function ProfileEditForm({ user }: ProfileEditFormProps) {
  const [isEditing, setIsEditing] = useState(false);
  const router = useRouter();

  const form = useForm<ProfileFormData>({
    resolver: zodResolver(profileSchema),
    defaultValues: {
      name: user.name ?? "",
      email: user.email ?? "",
    },
  });

  const updateProfile = api.users.update.useMutation({
    onSuccess: () => {
      setIsEditing(false);
      router.refresh();
    },
    onError: (error) => {
      console.error("Error updating profile:", error);
    },
  });

  const onSubmit = (data: ProfileFormData) => {
    updateProfile.mutate({
      id: user.id,
      name: data.name,
      email: data.email,
    });
  };

  const handleCancel = () => {
    form.reset();
    setIsEditing(false);
  };

  if (!isEditing) {
    return (
      <div className="space-y-4">
        <div className="grid grid-cols-2 gap-4">
          <div>
            <Label className="text-sm font-medium text-slate-700">Name</Label>
            <p className="mt-1 text-sm text-slate-900">
              {user.name ?? "Not set"}
            </p>
          </div>
          <div>
            <Label className="text-sm font-medium text-slate-700">Email</Label>
            <p className="mt-1 text-sm text-slate-900">
              {user.email ?? "Not set"}
            </p>
          </div>
        </div>

        <div className="flex justify-end">
          <Button onClick={() => setIsEditing(true)} variant="outline">
            Edit Profile
          </Button>
        </div>
      </div>
    );
  }

  return (
    <form onSubmit={form.handleSubmit(onSubmit)} className="space-y-4">
      {updateProfile.error && (
        <div className="rounded-md bg-red-50 p-3 text-sm text-red-700">
          <p className="font-medium">Error updating profile</p>
          <p>{updateProfile.error.message}</p>
        </div>
      )}

      <div className="grid grid-cols-1 gap-4 sm:grid-cols-2">
        <div className="space-y-2">
          <Label htmlFor="name">Full Name</Label>
          <Input
            id="name"
            {...form.register("name")}
            placeholder="Enter your full name"
            disabled={updateProfile.isPending}
          />
          {form.formState.errors.name && (
            <p className="text-sm text-red-600">
              {form.formState.errors.name.message}
            </p>
          )}
        </div>

        <div className="space-y-2">
          <Label htmlFor="email">Email Address</Label>
          <Input
            id="email"
            type="email"
            {...form.register("email")}
            placeholder="Enter your email address"
            disabled={updateProfile.isPending}
          />
          {form.formState.errors.email && (
            <p className="text-sm text-red-600">
              {form.formState.errors.email.message}
            </p>
          )}
        </div>
      </div>

      <div className="flex justify-end gap-3">
        <Button
          type="button"
          variant="outline"
          onClick={handleCancel}
          disabled={updateProfile.isPending}
        >
          Cancel
        </Button>
        <Button type="submit" disabled={updateProfile.isPending}>
          {updateProfile.isPending ? "Saving..." : "Save Changes"}
        </Button>
      </div>
    </form>
  );
}
