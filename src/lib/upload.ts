/**
 * Uploads a file to S3 and returns the public URL
 */
export async function uploadFile(file: File): Promise<string> {
  try {
    const formData = new FormData();
    formData.append("file", file);

    const response = await fetch("/api/upload", {
      method: "POST",
      body: formData,
    });

    if (!response.ok) {
      const errorData = await response.json().catch(() => ({}));
      console.error("Upload failed:", errorData);
      throw new Error(errorData.error || "Failed to upload file");
    }

    const data = await response.json();
    if (!data.url) {
      throw new Error("No URL returned from upload");
    }

    // Verify the URL is properly formed
    console.log("Upload successful, URL:", data.url);
    return data.url;
  } catch (error) {
    console.error("Error uploading file:", error);
    throw error;
  }
} 
