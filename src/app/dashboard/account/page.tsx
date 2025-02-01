"use client"

import { useSession } from "next-auth/react"
import { useForm } from "react-hook-form"
import { zodResolver } from "@hookform/resolvers/zod"
import { z } from "zod"
import { ImageIcon, Loader2 } from "lucide-react"
import Image from "next/image"
import { useState, useEffect } from "react"
import { useRouter } from "next/navigation"

import { Button } from "~/components/ui/button"
import {
  Card,
  CardContent,
  CardDescription,
  CardHeader,
  CardTitle,
} from "~/components/ui/card"
import {
  Form,
  FormControl,
  FormDescription,
  FormField,
  FormItem,
  FormLabel,
  FormMessage,
} from "~/components/ui/form"
import { Input } from "~/components/ui/input"
import { PageContent } from "~/components/layout/page-content"
import { PageHeader } from "~/components/layout/page-header"
import { api } from "~/trpc/react"
import { toast } from "sonner"
import { uploadFile } from "~/lib/upload"
import { ImageCropModal } from "~/components/ui/image-crop-modal"

const accountFormSchema = z.object({
  firstName: z.string().min(1, "First name is required"),
  lastName: z.string().min(1, "Last name is required"),
  email: z.string().email("Invalid email address").optional(),
  image: z.string().optional(),
})

type AccountFormValues = z.infer<typeof accountFormSchema>

export default function AccountPage() {
  const { data: session, update: updateSession } = useSession()
  const [cropFile, setCropFile] = useState<File | null>(null)
  const [isUploading, setIsUploading] = useState(false)
  const [previewImage, setPreviewImage] = useState<string | null>(null)
  const router = useRouter()

  // Debug full session object
  useEffect(() => {
    console.log("[Debug] Full session object:", JSON.stringify(session, null, 2));
  }, [session]);

  const form = useForm<AccountFormValues>({
    resolver: zodResolver(accountFormSchema),
    defaultValues: {
      firstName: session?.user?.name?.split(" ")[0] ?? "",
      lastName: session?.user?.name?.split(" ")[1] ?? "",
      email: session?.user?.email ?? "",
      image: session?.user?.image ?? undefined,
    }
  });

  const updateUser = api.user.update.useMutation();

  const onSubmit = async (data: AccountFormValues) => {
    try {
      console.log("[1] Starting update with form data:", data);

      // 1. Update database
      const result = await updateUser.mutateAsync({
        firstName: data.firstName,
        lastName: data.lastName,
        image: data.image ?? null,
      });
      console.log("[2] Database updated:", result);

      // 2. Show success message
      toast.success("Profile updated");
      console.log("[3] Showing success toast");

      // 3. Force a hard reload of the page
      console.log("[4] Forcing page reload");
      window.location.reload();
      
    } catch (error) {
      console.error("[X] Update failed:", error);
      toast.error("Failed to update profile");
    }
  };

  const handleCrop = async (blob: Blob) => {
    try {
      setIsUploading(true);
      
      const file = new File([blob], cropFile?.name ?? "avatar.jpg", {
        type: cropFile?.type ?? "image/jpeg",
      });
      
      const imageUrl = await uploadFile(file);
      
      setPreviewImage(imageUrl);
      form.setValue("image", imageUrl);
      
    } catch (error) {
      console.error("Error uploading file:", error);
      toast.error("Error uploading image");
      setPreviewImage(null);
      form.setValue("image", undefined);
    } finally {
      setIsUploading(false);
      setCropFile(null);
    }
  };

  return (
    <>
      <PageHeader
        title="Account Settings"
        description="Manage your profile information"
      />
      <PageContent>
        <div className="grid gap-6">
          <Card>
            <CardHeader>
              <CardTitle>Profile</CardTitle>
              <CardDescription>
                Update your profile picture and information
              </CardDescription>
            </CardHeader>
            <CardContent>
              <Form {...form}>
                <form onSubmit={form.handleSubmit(onSubmit)} className="space-y-8">
                  <div className="flex flex-col gap-8 sm:flex-row">
                    <FormField
                      control={form.control}
                      name="image"
                      render={({ field }) => (
                        <FormItem className="flex flex-col items-start justify-start">
                          <FormLabel className="group flex size-32 cursor-pointer items-center justify-center overflow-hidden rounded-full border bg-muted transition-colors hover:bg-muted/80">
                            {previewImage ? (
                              <div className="relative size-full overflow-hidden rounded-full">
                                <div className="absolute inset-0 z-10 flex items-center justify-center bg-black/60 opacity-0 transition-opacity group-hover:opacity-100">
                                  <p className="text-xs font-medium text-white">Change Image</p>
                                </div>
                                <Image
                                  src={previewImage}
                                  alt="Avatar"
                                  fill
                                  sizes="128px"
                                  className="object-cover"
                                  priority
                                  onError={(e) => {
                                    console.error("Error loading image:", previewImage);
                                    e.currentTarget.style.display = "none";
                                    setPreviewImage(null);
                                    form.setValue("image", undefined);
                                  }}
                                />
                              </div>
                            ) : (
                              <ImageIcon className="size-8 text-muted-foreground transition-colors group-hover:text-muted-foreground/80" />
                            )}
                            <FormControl>
                              <Input
                                type="file"
                                accept="image/*"
                                className="hidden"
                                onChange={(e) => {
                                  const file = e.target.files?.[0];
                                  if (file) {
                                    setCropFile(file);
                                  }
                                }}
                                disabled={isUploading}
                              />
                            </FormControl>
                          </FormLabel>
                          <FormDescription>
                            Click to upload a new profile picture
                          </FormDescription>
                          <FormMessage />
                        </FormItem>
                      )}
                    />
                    <div className="flex-1 space-y-4">
                      <div className="grid gap-4 sm:grid-cols-2">
                        <FormField
                          control={form.control}
                          name="firstName"
                          render={({ field }) => (
                            <FormItem>
                              <FormLabel>First Name</FormLabel>
                              <FormControl>
                                <Input
                                  placeholder="Enter your first name"
                                  {...field}
                                  disabled={updateUser.isPending}
                                />
                              </FormControl>
                              <FormMessage />
                            </FormItem>
                          )}
                        />
                        <FormField
                          control={form.control}
                          name="lastName"
                          render={({ field }) => (
                            <FormItem>
                              <FormLabel>Last Name</FormLabel>
                              <FormControl>
                                <Input
                                  placeholder="Enter your last name"
                                  {...field}
                                  disabled={updateUser.isPending}
                                />
                              </FormControl>
                              <FormMessage />
                            </FormItem>
                          )}
                        />
                      </div>
                      <FormField
                        control={form.control}
                        name="email"
                        render={({ field }) => (
                          <FormItem>
                            <FormLabel>Email</FormLabel>
                            <FormControl>
                              <Input
                                type="email"
                                placeholder="Enter your email"
                                {...field}
                                disabled
                              />
                            </FormControl>
                            <FormDescription>
                              Email cannot be changed
                            </FormDescription>
                            <FormMessage />
                          </FormItem>
                        )}
                      />
                    </div>
                  </div>
                  <div className="flex justify-end">
                    <Button
                      type="submit"
                      disabled={updateUser.isPending || isUploading}
                    >
                      {(updateUser.isPending || isUploading) && (
                        <Loader2 className="mr-2 size-4 animate-spin" />
                      )}
                      Save Changes
                    </Button>
                  </div>
                </form>
              </Form>
            </CardContent>
          </Card>
        </div>
      </PageContent>
      {cropFile && (
        <ImageCropModal
          file={cropFile}
          aspect={1}
          onCrop={handleCrop}
          onCancel={() => setCropFile(null)}
          className="sm:max-w-md"
          cropBoxClassName="rounded-full border-2 border-primary shadow-2xl"
          overlayClassName="bg-background/80 backdrop-blur-sm"
        />
      )}
    </>
  )
} 
