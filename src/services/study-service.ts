export class StudyService {
  constructor(
    private db: DbClient,
    private storage: StorageClient
  ) {}

  async createStudy(params: CreateStudyParams) {
    return this.db.transaction(async (tx) => {
      const study = await tx.insert(studies).values(params).returning()
      await this.storage.putObject("studies", study.id, params.config)
      return study
    })
  }
} 