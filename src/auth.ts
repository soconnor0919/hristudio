import { Auth } from "@auth/core"
import Credentials from "@auth/core/providers/credentials"

export const authOptions: AuthConfig = {
  providers: [
    Credentials({
      credentials: {
        email: { label: "Email", type: "email" },
        password: { label: "Password", type: "password" }
      },
      async authorize(credentials) {
        // Drizzle ORM user lookup
        const user = await db.query.users.findFirst({
          where: (users, { eq }) => eq(users.email, credentials.email)
        })
        return verifyPassword(credentials.password, user?.password) ? user : null
      }
    })
  ],
  session: { strategy: "jwt" },
  adapter: DrizzleAdapter(db)
} 