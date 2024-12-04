import { Suspense } from "react";
import { Loader2 } from "lucide-react";
import { InvitationAcceptContent } from "./invitation-accept-content";

interface InvitationAcceptPageProps {
  params: { token: string };
}

export default async function InvitationAcceptPage({ params }: InvitationAcceptPageProps) {
  const token = await Promise.resolve(params.token);
  
  return (
    <Suspense
      fallback={
        <div className="flex items-center justify-center min-h-screen">
          <Loader2 className="h-8 w-8 animate-spin" />
        </div>
      }
    >
      <InvitationAcceptContent token={token} />
    </Suspense>
  );
} 