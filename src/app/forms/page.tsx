import Layout from "~/components/layout";
import { FormsGrid } from "~/components/forms/FormsGrid";
import { UploadFormButton } from "~/components/forms/UploadFormButton";
import { Card, CardHeader, CardTitle, CardContent } from "~/components/ui/card";

export default function FormsPage() {
  return (
    <Layout pageTitle="Forms">
      <Card>
        <CardHeader>
          <CardTitle className="flex justify-between items-center">
            <span>Forms</span>
            <UploadFormButton />
          </CardTitle>
        </CardHeader>
        <CardContent>
          <FormsGrid />
        </CardContent>
      </Card>
    </Layout>
  );
}