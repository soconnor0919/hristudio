"use client";

import { User } from "next-auth";
import { Avatar, AvatarFallback } from "~/components/ui/avatar";
import Image from "next/image";

interface UserAvatarProps {
  user: Pick<User, "name" | "image">;
  className?: string;
}

export function UserAvatar({ user, className }: UserAvatarProps) {
  return (
    <Avatar className={className}>
      {user.image ? (
        <div className="relative size-full">
          <Image
            alt={user.name ?? "Avatar"}
            src={user.image}
            fill
            sizes="32px"
            className="rounded-full object-cover"
            onError={(e) => {
              console.error("Error loading avatar image:", user.image);
              e.currentTarget.style.display = "none";
            }}
          />
        </div>
      ) : null}
      <AvatarFallback>
        {user.name?.charAt(0).toUpperCase() ?? "?"}
      </AvatarFallback>
    </Avatar>
  );
} 