"use client";

import Layout from "~/components/layout";
import { Card, CardHeader, CardTitle, CardContent } from "~/components/ui/card";
import { useStudyContext } from '~/context/StudyContext';
import { useEffect, useState } from 'react';
import Link from 'next/link';
import { Button } from "~/components/ui/button";
import { Avatar, AvatarFallback } from "~/components/ui/avatar";

interface ParticipantWithTrial {
  id: number;
  name: string;
  latestTrialTimestamp: string | null;
  createdAt: string; // Add createdAt to the interface
}

const DashboardPage: React.FC = () => {
  const { selectedStudy } = useStudyContext();
  const [participants, setParticipants] = useState<ParticipantWithTrial[]>([]);

  useEffect(() => {
    const fetchParticipants = async () => {
      if (selectedStudy) {
        const response = await fetch(`/api/participants?studyId=${selectedStudy.id}`);
        const data = await response.json();
        setParticipants(data);
      }
    };

    fetchParticipants();
  }, [selectedStudy]);

  const formatDate = (dateString: string | null) => {
    if (!dateString) return 'No trials yet';
    const date = new Date(dateString);
    return date.toLocaleDateString() + ' ' + date.toLocaleTimeString();
  };

  return (
    <Layout pageTitle="Dashboard">
      <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
        <Card className="card-level-1">
          <CardHeader>
            <CardTitle>Platform Information</CardTitle>
          </CardHeader>
          <CardContent>
            {/* Add content for Platform Information */}
          </CardContent>
        </Card>
        <Card className="card-level-1">
          <CardHeader>
            <CardTitle>Participants</CardTitle>
          </CardHeader>
          <CardContent>
            <div className="grid grid-cols-1 gap-4">
              {participants.slice(0, 4).map(participant => (
                <Card key={participant.id} className="card-level-2 p-3 px-4 flex items-center">
                  <Avatar className="mr-4">
                    <AvatarFallback>{participant.name.split(' ').map(n => n[0]).join('')}</AvatarFallback>
                  </Avatar>
                  <div className="flex-1">
                    <h3 className="font-semibold">{participant.name}</h3>
                    <p className="text-sm text-muted-foreground">
                      Last trial: {formatDate(participant.latestTrialTimestamp)}
                    </p>
                  </div>
                </Card>
              ))}
            </div>
            {participants.length > 4 && (
              <div className="mt-4 text-center">
                <Link href="/participants">
                  <Button variant="outline" className="text-blue-600 hover:underline">
                    View More Participants
                  </Button>
                </Link>
              </div>
            )}
          </CardContent>
        </Card>
        <Card>
          <CardHeader>
            <CardTitle>Project Members</CardTitle>
          </CardHeader>
          <CardContent>
            {/* Add content for Project Members */}
          </CardContent>
        </Card>
        <Card>
          <CardHeader>
            <CardTitle>Completed Trials</CardTitle>
          </CardHeader>
          <CardContent>
            {/* Add content for Completed Trials */}
          </CardContent>
        </Card>
      </div>
    </Layout>
  );
};

export default DashboardPage;