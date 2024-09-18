# Use the Node.js 18 Alpine Linux image as the base image
FROM node:22-alpine 

# Set the working directory inside the container to /app
WORKDIR /app

# Copy package.json and package-lock.json files into the working directory
COPY package*.json ./

# Install the dependencies specified in package.json
RUN npm install -g pnpm
RUN pnpm install

# Copy all the files from the local directory to the working directory in the container
COPY . .

# Push database schema to database
RUN pnpm drizzle-kit push

# Run the application in development mode
CMD ["pnpm", "run", "dev"]