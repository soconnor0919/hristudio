import { useUser } from "@clerk/nextjs";
import { useEffect, useState } from "react";
import { PERMISSIONS, type PermissionCode } from "~/lib/permissions";

export function usePermissions() {
  const { user } = useUser();
  const [permissions, setPermissions] = useState<Set<string>>(new Set());
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    async function fetchPermissions() {
      if (!user?.id) return;
      
      try {
        const response = await fetch('/api/permissions');
        const data = await response.json();
        setPermissions(new Set(data));
      } catch (error) {
        console.error('Error fetching permissions:', error);
      } finally {
        setLoading(false);
      }
    }

    fetchPermissions();
  }, [user?.id]);

  const hasPermission = (permission: PermissionCode) => {
    return permissions.has(PERMISSIONS[permission]);
  };

  return {
    hasPermission,
    loading,
  };
}
