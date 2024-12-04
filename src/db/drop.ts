import { sql } from '@vercel/postgres';
import { db } from './index';
import { config } from 'dotenv';

// load .env.local
config({ path: '.env.local' });

async function dropAllTables() {
  try {
    // drop all tables, regardless of name
    await sql`
      DROP TABLE IF EXISTS ${sql.raw(Object.values(tables).map(table => table.name).join(', '))}
    `;
    console.log('All tables dropped successfully');
  } catch (error) {
    console.error('Error dropping tables:', error);
    process.exit(1);
  }
}

dropAllTables(); 