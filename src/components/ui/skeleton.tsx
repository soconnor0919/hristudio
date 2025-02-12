import { cn } from "~/lib/utils"
import { Card, CardContent, CardHeader } from "~/components/ui/card"

function Skeleton({
  className,
  ...props
}: React.HTMLAttributes<HTMLDivElement>) {
  return (
    <div
      className={cn("animate-pulse rounded-md bg-muted", className)}
      {...props}
    />
  )
}

function TableRowSkeleton() {
  return (
    <div className="flex space-x-4 p-4">
      <Skeleton className="h-5 w-[20%]" />
      <Skeleton className="h-5 w-[15%]" />
      <Skeleton className="h-5 w-[30%]" />
      <Skeleton className="h-5 w-[35%]" />
    </div>
  )
}

function TableSkeleton() {
  return (
    <div className="space-y-3">
      <div className="flex space-x-4 p-4 border-b">
        <Skeleton className="h-4 w-[20%]" />
        <Skeleton className="h-4 w-[15%]" />
        <Skeleton className="h-4 w-[30%]" />
        <Skeleton className="h-4 w-[35%]" />
      </div>
      {[...Array(5)].map((_, i) => (
        <TableRowSkeleton key={i} />
      ))}
    </div>
  )
}

function CardSkeleton() {
  return (
    <div className="p-6 space-y-4">
      <Skeleton className="h-7 w-[40%]" />
      <Skeleton className="h-4 w-[60%]" />
      <div className="pt-4">
        <Skeleton className="h-4 w-[25%]" />
      </div>
    </div>
  )
}

function StudyListSkeleton() {
  return (
    <div className="space-y-6">
      {[...Array(3)].map((_, i) => (
        <div key={i} className="rounded-lg border bg-card">
          <CardSkeleton />
        </div>
      ))}
    </div>
  )
}

function StudyDetailsSkeleton() {
  return (
    <div className="space-y-6">
      {/* Overview Card */}
      <Card>
        <CardHeader>
          <Skeleton className="h-7 w-[150px] mb-2" />
          <Skeleton className="h-4 w-[250px]" />
        </CardHeader>
        <CardContent>
          <div className="grid gap-4 sm:grid-cols-2">
            <div>
              <Skeleton className="h-4 w-[100px] mb-1" />
              <Skeleton className="h-5 w-[150px]" />
            </div>
            <div>
              <Skeleton className="h-4 w-[100px] mb-1" />
              <Skeleton className="h-5 w-[200px]" />
            </div>
          </div>
        </CardContent>
      </Card>

      {/* Stats Cards */}
      <div className="grid gap-4 md:grid-cols-3">
        {[...Array(3)].map((_, i) => (
          <Card key={i}>
            <CardHeader className="flex flex-row items-center justify-between space-y-0 pb-2">
              <Skeleton className="h-4 w-[120px]" />
              <Skeleton className="h-4 w-4" />
            </CardHeader>
            <CardContent>
              <Skeleton className="h-7 w-[60px] mb-1" />
              <Skeleton className="h-4 w-[100px]" />
            </CardContent>
          </Card>
        ))}
      </div>

      {/* Activity Card */}
      <Card>
        <CardHeader>
          <Skeleton className="h-7 w-[150px] mb-2" />
          <Skeleton className="h-4 w-[200px]" />
        </CardHeader>
        <CardContent>
          <div className="space-y-4">
            {[...Array(3)].map((_, i) => (
              <div key={i} className="flex gap-4">
                <Skeleton className="h-8 w-8 rounded-full" />
                <div className="flex-1 space-y-2">
                  <Skeleton className="h-4 w-[150px]" />
                  <Skeleton className="h-4 w-[250px]" />
                  <Skeleton className="h-3 w-[100px]" />
                </div>
              </div>
            ))}
          </div>
        </CardContent>
      </Card>
    </div>
  )
}

export { 
  Skeleton,
  TableRowSkeleton,
  TableSkeleton,
  CardSkeleton,
  StudyListSkeleton,
  StudyDetailsSkeleton,
}
