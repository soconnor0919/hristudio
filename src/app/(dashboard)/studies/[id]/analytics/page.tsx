"use client";

import { useParams } from "next/navigation";
import { Suspense, useEffect } from "react";
import {
  Activity,
  BarChart3,
  Calendar,
  Download,
  Filter,
  TrendingDown,
  TrendingUp,
} from "lucide-react";

import { Button } from "~/components/ui/button";
import {
  Card,
  CardContent,
  CardDescription,
  CardHeader,
  CardTitle,
} from "~/components/ui/card";
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from "~/components/ui/select";
import { ManagementPageLayout } from "~/components/ui/page-layout";
import { useStudyContext } from "~/lib/study-context";
import { useSelectedStudyDetails } from "~/hooks/useSelectedStudyDetails";

// Mock chart component - replace with actual charting library
function MockChart({ title, data }: { title: string; data: number[] }) {
  const maxValue = Math.max(...data);

  return (
    <div className="space-y-2">
      <h4 className="text-sm font-medium">{title}</h4>
      <div className="flex h-32 items-end space-x-1">
        {data.map((value, index) => (
          <div
            key={index}
            className="bg-primary min-h-[4px] flex-1 rounded-t"
            style={{ height: `${(value / maxValue) * 100}%` }}
          />
        ))}
      </div>
    </div>
  );
}

function AnalyticsOverview() {
  const metrics = [
    {
      title: "Total Trials This Month",
      value: "142",
      change: "+12%",
      trend: "up",
      description: "vs last month",
      icon: Activity,
    },
    {
      title: "Avg Trial Duration",
      value: "24.5m",
      change: "-3%",
      trend: "down",
      description: "vs last month",
      icon: Calendar,
    },
    {
      title: "Completion Rate",
      value: "94.2%",
      change: "+2.1%",
      trend: "up",
      description: "vs last month",
      icon: TrendingUp,
    },
    {
      title: "Participant Retention",
      value: "87.3%",
      change: "+5.4%",
      trend: "up",
      description: "vs last month",
      icon: BarChart3,
    },
  ];

  return (
    <div className="grid gap-4 md:grid-cols-2 lg:grid-cols-4">
      {metrics.map((metric) => (
        <Card key={metric.title}>
          <CardHeader className="flex flex-row items-center justify-between space-y-0 pb-2">
            <CardTitle className="text-sm font-medium">
              {metric.title}
            </CardTitle>
            <metric.icon className="text-muted-foreground h-4 w-4" />
          </CardHeader>
          <CardContent>
            <div className="text-2xl font-bold">{metric.value}</div>
            <div className="text-muted-foreground flex items-center space-x-2 text-xs">
              <span
                className={`flex items-center ${
                  metric.trend === "up" ? "text-green-600" : "text-red-600"
                }`}
              >
                {metric.trend === "up" ? (
                  <TrendingUp className="mr-1 h-3 w-3" />
                ) : (
                  <TrendingDown className="mr-1 h-3 w-3" />
                )}
                {metric.change}
              </span>
              <span>{metric.description}</span>
            </div>
          </CardContent>
        </Card>
      ))}
    </div>
  );
}

function ChartsSection() {
  const trialData = [12, 19, 15, 27, 32, 28, 35, 42, 38, 41, 37, 44];
  const participantData = [8, 12, 10, 15, 18, 16, 20, 24, 22, 26, 23, 28];
  const completionData = [85, 88, 92, 89, 94, 91, 95, 92, 96, 94, 97, 94];

  return (
    <div className="grid gap-4 lg:grid-cols-3">
      <Card>
        <CardHeader>
          <CardTitle>Trial Volume</CardTitle>
          <CardDescription>Monthly trial execution trends</CardDescription>
        </CardHeader>
        <CardContent>
          <MockChart title="Trials per Month" data={trialData} />
        </CardContent>
      </Card>

      <Card>
        <CardHeader>
          <CardTitle>Participant Enrollment</CardTitle>
          <CardDescription>New participants over time</CardDescription>
        </CardHeader>
        <CardContent>
          <MockChart title="New Participants" data={participantData} />
        </CardContent>
      </Card>

      <Card>
        <CardHeader>
          <CardTitle>Completion Rates</CardTitle>
          <CardDescription>Trial completion percentage</CardDescription>
        </CardHeader>
        <CardContent>
          <MockChart title="Completion %" data={completionData} />
        </CardContent>
      </Card>
    </div>
  );
}

function RecentInsights() {
  const insights = [
    {
      title: "Peak Performance Hours",
      description:
        "Participants show 23% better performance during 10-11 AM trials",
      type: "trend",
      severity: "info",
    },
    {
      title: "Attention Span Decline",
      description:
        "Average attention span has decreased by 8% over the last month",
      type: "alert",
      severity: "warning",
    },
    {
      title: "High Completion Rate",
      description: "Memory retention study achieved 98% completion rate",
      type: "success",
      severity: "success",
    },
    {
      title: "Equipment Utilization",
      description: "Robot interaction trials are at 85% capacity utilization",
      type: "info",
      severity: "info",
    },
  ];

  const getSeverityColor = (severity: string) => {
    switch (severity) {
      case "success":
        return "bg-green-50 text-green-700 border-green-200";
      case "warning":
        return "bg-yellow-50 text-yellow-700 border-yellow-200";
      case "info":
        return "bg-blue-50 text-blue-700 border-blue-200";
      default:
        return "bg-gray-50 text-gray-700 border-gray-200";
    }
  };

  return (
    <Card>
      <CardHeader>
        <CardTitle>Recent Insights</CardTitle>
        <CardDescription>
          AI-generated insights from your research data
        </CardDescription>
      </CardHeader>
      <CardContent>
        <div className="space-y-4">
          {insights.map((insight, index) => (
            <div
              key={index}
              className={`rounded-lg border p-4 ${getSeverityColor(insight.severity)}`}
            >
              <h4 className="mb-1 font-medium">{insight.title}</h4>
              <p className="text-sm">{insight.description}</p>
            </div>
          ))}
        </div>
      </CardContent>
    </Card>
  );
}

function AnalyticsContent({ studyId: _studyId }: { studyId: string }) {
  return (
    <div className="space-y-6">
      {/* Header with time range controls */}
      <div className="flex items-center justify-between">
        <div className="flex items-center space-x-2">
          <Select defaultValue="30d">
            <SelectTrigger className="w-[120px]">
              <SelectValue placeholder="Time range" />
            </SelectTrigger>
            <SelectContent>
              <SelectItem value="7d">Last 7 days</SelectItem>
              <SelectItem value="30d">Last 30 days</SelectItem>
              <SelectItem value="90d">Last 90 days</SelectItem>
              <SelectItem value="1y">Last year</SelectItem>
            </SelectContent>
          </Select>
          <Button variant="outline" size="sm">
            <Filter className="mr-2 h-4 w-4" />
            Filter
          </Button>
          <Button variant="outline" size="sm">
            <Download className="mr-2 h-4 w-4" />
            Export
          </Button>
        </div>
      </div>

      {/* Overview Metrics */}
      <AnalyticsOverview />

      {/* Charts */}
      <ChartsSection />

      {/* Insights */}
      <div className="grid gap-4 lg:grid-cols-3">
        <div className="lg:col-span-2">
          <RecentInsights />
        </div>
        <Card>
          <CardHeader>
            <CardTitle>Quick Actions</CardTitle>
            <CardDescription>Generate custom reports</CardDescription>
          </CardHeader>
          <CardContent className="space-y-2">
            <Button variant="outline" className="w-full justify-start">
              <BarChart3 className="mr-2 h-4 w-4" />
              Trial Performance Report
            </Button>
            <Button variant="outline" className="w-full justify-start">
              <Activity className="mr-2 h-4 w-4" />
              Participant Engagement
            </Button>
            <Button variant="outline" className="w-full justify-start">
              <TrendingUp className="mr-2 h-4 w-4" />
              Trend Analysis
            </Button>
            <Button variant="outline" className="w-full justify-start">
              <Download className="mr-2 h-4 w-4" />
              Custom Export
            </Button>
          </CardContent>
        </Card>
      </div>
    </div>
  );
}

export default function StudyAnalyticsPage() {
  const params = useParams();
  const studyId: string = typeof params.id === "string" ? params.id : "";
  const { setSelectedStudyId, selectedStudyId } = useStudyContext();
  const { study } = useSelectedStudyDetails();

  // Set the active study if it doesn't match the current route
  useEffect(() => {
    if (studyId && selectedStudyId !== studyId) {
      setSelectedStudyId(studyId);
    }
  }, [studyId, selectedStudyId, setSelectedStudyId]);

  return (
    <ManagementPageLayout
      title="Analytics"
      description="Insights and data analysis for this study"
      breadcrumb={[
        { label: "Dashboard", href: "/dashboard" },
        { label: "Studies", href: "/studies" },
        { label: study?.name ?? "Study", href: `/studies/${studyId}` },
        { label: "Analytics" },
      ]}
    >
      <Suspense fallback={<div>Loading analytics...</div>}>
        <AnalyticsContent studyId={studyId} />
      </Suspense>
    </ManagementPageLayout>
  );
}
