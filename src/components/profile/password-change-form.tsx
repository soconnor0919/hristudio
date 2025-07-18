"use client";

import { useState } from "react";
import { useForm } from "react-hook-form";
import { zodResolver } from "@hookform/resolvers/zod";
import { z } from "zod";
import { Button } from "~/components/ui/button";
import { Input } from "~/components/ui/input";
import { Label } from "~/components/ui/label";
import { api } from "~/trpc/react";

const passwordSchema = z
  .object({
    currentPassword: z.string().min(1, "Current password is required"),
    newPassword: z
      .string()
      .min(6, "Password must be at least 6 characters")
      .max(100, "Password is too long"),
    confirmPassword: z.string().min(1, "Please confirm your new password"),
  })
  .refine((data) => data.newPassword === data.confirmPassword, {
    message: "Passwords don't match",
    path: ["confirmPassword"],
  });

type PasswordFormData = z.infer<typeof passwordSchema>;

export function PasswordChangeForm() {
  const [showForm, setShowForm] = useState(false);

  const form = useForm<PasswordFormData>({
    resolver: zodResolver(passwordSchema),
    defaultValues: {
      currentPassword: "",
      newPassword: "",
      confirmPassword: "",
    },
  });

  const changePassword = api.users.changePassword.useMutation({
    onSuccess: () => {
      form.reset();
      setShowForm(false);
    },
    onError: (error) => {
      console.error("Error changing password:", error);
    },
  });

  const onSubmit = (data: PasswordFormData) => {
    changePassword.mutate({
      currentPassword: data.currentPassword,
      newPassword: data.newPassword,
    });
  };

  const handleCancel = () => {
    form.reset();
    setShowForm(false);
  };

  if (!showForm) {
    return (
      <div className="space-y-4">
        <div>
          <p className="text-sm text-slate-600">
            Change your account password for enhanced security
          </p>
        </div>

        <div className="flex justify-end">
          <Button onClick={() => setShowForm(true)} variant="outline">
            Change Password
          </Button>
        </div>
      </div>
    );
  }

  return (
    <form onSubmit={form.handleSubmit(onSubmit)} className="space-y-4">
      {changePassword.error && (
        <div className="rounded-md bg-red-50 p-3 text-sm text-red-700">
          <p className="font-medium">Error changing password</p>
          <p>{changePassword.error.message}</p>
        </div>
      )}

      {changePassword.isSuccess && (
        <div className="rounded-md bg-green-50 p-3 text-sm text-green-700">
          <p className="font-medium">Password changed successfully</p>
          <p>Your password has been updated.</p>
        </div>
      )}

      <div className="space-y-4">
        <div className="space-y-2">
          <Label htmlFor="currentPassword">Current Password</Label>
          <Input
            id="currentPassword"
            type="password"
            {...form.register("currentPassword")}
            placeholder="Enter your current password"
            disabled={changePassword.isPending}
          />
          {form.formState.errors.currentPassword && (
            <p className="text-sm text-red-600">
              {form.formState.errors.currentPassword.message}
            </p>
          )}
        </div>

        <div className="space-y-2">
          <Label htmlFor="newPassword">New Password</Label>
          <Input
            id="newPassword"
            type="password"
            {...form.register("newPassword")}
            placeholder="Enter your new password"
            disabled={changePassword.isPending}
          />
          {form.formState.errors.newPassword && (
            <p className="text-sm text-red-600">
              {form.formState.errors.newPassword.message}
            </p>
          )}
        </div>

        <div className="space-y-2">
          <Label htmlFor="confirmPassword">Confirm New Password</Label>
          <Input
            id="confirmPassword"
            type="password"
            {...form.register("confirmPassword")}
            placeholder="Confirm your new password"
            disabled={changePassword.isPending}
          />
          {form.formState.errors.confirmPassword && (
            <p className="text-sm text-red-600">
              {form.formState.errors.confirmPassword.message}
            </p>
          )}
        </div>
      </div>

      <div className="flex justify-end gap-3">
        <Button
          type="button"
          variant="outline"
          onClick={handleCancel}
          disabled={changePassword.isPending}
        >
          Cancel
        </Button>
        <Button type="submit" disabled={changePassword.isPending}>
          {changePassword.isPending ? "Changing..." : "Change Password"}
        </Button>
      </div>
    </form>
  );
}
