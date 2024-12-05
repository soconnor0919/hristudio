import { Avatar, AvatarFallback, AvatarImage } from "~/components/ui/avatar";

interface UserAvatarProps {
  user: {
    name?: string | null;
    email: string;
    imageUrl?: string | null;
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
      {user.imageUrl && <AvatarImage src={user.imageUrl} alt={user.name || user.email} />}
      <AvatarFallback>{initials}</AvatarFallback>
    </Avatar>
  );
} 