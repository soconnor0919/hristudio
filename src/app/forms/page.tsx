import Layout from "~/components/layout";
import { FormsGrid } from "~/components/forms/FormsGrid";
import { UploadFormButton } from "~/components/forms/UploadFormButton";

export default function FormsPage() {
  return (
    <Layout>
      <div className="flex justify-between items-center mb-6">
        <h1 className="text-3xl font-bold">Forms</h1>
        <UploadFormButton />
      </div>
      <FormsGrid />
    </Layout>
  );
}