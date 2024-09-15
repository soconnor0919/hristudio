import Link from 'next/link';
import Image from 'next/image';
import { auth } from "@clerk/nextjs/server";
import { redirect } from "next/navigation";

export default function HomePage() {
  const { userId } = auth();

  if (userId) {
    redirect("/dash");
  }

  return (
    <div className="min-h-screen bg-gradient-to-b from-blue-100 to-white">
      <div className="container mx-auto px-4 py-16">
        <header className="text-center mb-16">
          <h1 className="text-5xl font-bold mb-4 text-blue-800">Welcome to HRIStudio</h1>
          <p className="text-xl text-gray-600 max-w-3xl mx-auto">
            Empowering Human-Robot Interaction Research and Development
          </p>
        </header>

        <div className="grid md:grid-cols-2 gap-12 items-center mb-16">
          <div>
            <h2 className="text-3xl font-semibold mb-4 text-blue-700">About HRIStudio</h2>
            <p className="text-lg text-gray-700 mb-4">
              HRIStudio is a cutting-edge platform designed to streamline the process of creating, 
              managing, and analyzing Human-Robot Interaction experiments. Our suite of tools 
              empowers researchers and developers to push the boundaries of HRI research.
            </p>
            <p className="text-lg text-gray-700 mb-4">
              With HRIStudio, you can:
            </p>
            <ul className="list-disc list-inside text-gray-700 mb-6">
              <li>Design complex interaction scenarios with ease</li>
              <li>Collect and analyze data in real-time</li>
              <li>Collaborate seamlessly with team members</li>
              <li>Visualize results with advanced reporting tools</li>
            </ul>
          </div>
          <div className="relative aspect-video w-full">
            <Image
              src="/hristudio_laptop.png"
              alt="HRIStudio Interface on Laptop"
              fill
              style={{ objectFit: 'contain' }}
              // className="rounded-lg shadow-lg"
            />
          </div>
        </div>

        <div className="text-center mb-12">
          <h2 className="text-3xl font-semibold mb-4 text-blue-700">Join the HRI Revolution</h2>
          <p className="text-lg text-gray-700 mb-6">
            Whether you're a seasoned researcher or just starting in the field of Human-Robot Interaction, 
            HRIStudio provides the tools and support you need to succeed.
          </p>
          <div className="space-x-4">
            <Link href="/sign-in" className="bg-blue-600 hover:bg-blue-700 text-white font-bold py-3 px-6 rounded-full transition duration-300">
              Sign In
            </Link>
            <Link href="/sign-up" className="bg-green-600 hover:bg-green-700 text-white font-bold py-3 px-6 rounded-full transition duration-300">
              Sign Up
            </Link>
          </div>
        </div>

        <footer className="text-center text-gray-600">
          <p>© 2024 HRIStudio. All rights reserved.</p>
        </footer>
      </div>
    </div>
  );
}