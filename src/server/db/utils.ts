import { pgTableCreator } from "drizzle-orm/pg-core";

/**
 * This creates tables with the given prefix to avoid naming conflicts in the database
 * @see https://orm.drizzle.team/docs/goodies#multi-project-schema
 */
export const createTable = pgTableCreator((name) => `hs_${name}`); 