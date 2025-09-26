import type { Metadata } from "next";
import { Geist, Geist_Mono } from "next/font/google";
import { RobotProvider } from "@/contexts/RobotContext";
import "./globals.css";

const geistSans = Geist({
  variable: "--font-geist-sans",
  subsets: ["latin"],
});

const geistMono = Geist_Mono({
  variable: "--font-geist-mono",
  subsets: ["latin"],
});

export const metadata: Metadata = {
  title: "C38 Robot Controller",
  description: "Control interface for C38 robot arm",
};

export default function RootLayout({
  children,
}: Readonly<{
  children: React.ReactNode;
}>) {
  return (
    <html lang="en">
      <body
        className={`${geistSans.variable} ${geistMono.variable} antialiased`}
      >
        <RobotProvider>
          {children}
        </RobotProvider>
      </body>
    </html>
  );
}
