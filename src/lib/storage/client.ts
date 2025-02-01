interface StorageClient {
  putObject(bucket: string, key: string, data: Buffer): Promise<string>;
  getObject(bucket: string, key: string): Promise<Buffer>;
  deleteObject(bucket: string, key: string): Promise<void>;
}

export class S3StorageClient implements StorageClient {
  constructor(private client: S3Client) {}
  // Implement S3-specific methods
}

export class MinioStorageClient implements StorageClient {
  constructor(private client: MinioClient) {}
  // Implement Minio-specific methods
} 