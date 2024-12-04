import { Avatar, AvatarFallback } from "~/components/ui/avatar";

interface UserAvatarProps {
  user: {
    name?: string | null;
    email: string;
  };
  className?: string;
}

export function UserAvatar({ user, className }: UserAvatarProps) {
  function getInitials(name: string) {
    return name
      .split(' ')
      .map(part => part[0])
      .join('')
      .toUpperCase();
  }

  const initials = user.name ? getInitials(user.name) : user.email[0].toUpperCase();

  return (
    <Avatar className={className}>
      <AvatarFallback>{initials}</AvatarFallback>
    </Avatar>
  );
} 