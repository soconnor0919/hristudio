import { sql } from '@vercel/postgres';
import { db } from './index';
import { config } from 'dotenv';

// load .env.local
config({ path: '.env.local' });

async function dropAllTables() {
  try {
    await sql`
      DROP TABLE IF EXISTS 
        user_roles,
        role_permissions,
        permissions,
        roles,
        participant,
        study,
        users
      CASCADE;
    `;
    console.log('All tables dropped successfully');
  } catch (error) {
    console.error('Error dropping tables:', error);
    process.exit(1);
  }
}

dropAllTables(); 