import Layout from "~/components/layout";
import { Card, CardHeader, CardTitle, CardContent } from "~/components/ui/card";

const DashboardPage: React.FC = () => {
  return (
    <Layout>
      <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
        <Card>
          <CardHeader>
            <CardTitle>Platform Information</CardTitle>
          </CardHeader>
          <CardContent>
            {/* Add content for Platform Information */}
          </CardContent>
        </Card>
        <Card>
          <CardHeader>
            <CardTitle>Participants</CardTitle>
          </CardHeader>
          <CardContent>
            {/* Add content for Participants */}
          </CardContent>
        </Card>
        <Card>
          <CardHeader>
            <CardTitle>Project Members</CardTitle>
          </CardHeader>
          <CardContent>
            {/* Add content for Project Members */}
          </CardContent>
        </Card>
        <Card>
          <CardHeader>
            <CardTitle>Completed Trials</CardTitle>
          </CardHeader>
          <CardContent>
            {/* Add content for Completed Trials */}
          </CardContent>
        </Card>
      </div>
    </Layout>
  );
};

export default DashboardPage;