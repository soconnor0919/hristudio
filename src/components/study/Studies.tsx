"use client";

import React from 'react';
import { Card, CardContent, CardHeader, CardTitle } from "~/components/ui/card";
import { useStudies } from '~/hooks/useStudies';
import { Button } from "~/components/ui/button";

export function Studies() {
  const { studies, deleteStudy } = useStudies();

  return (
    <div className="space-y-4">
      {studies.map((study) => (
        <Card key={study.id}>
          <CardHeader>
            <CardTitle>{study.title}</CardTitle>
          </CardHeader>
          <CardContent>
            <p>{study.description}</p>
            <div className="flex space-x-2 mt-2">
              <Button variant="destructive" onClick={() => deleteStudy(study.id)}>Delete</Button>
            </div>
          </CardContent>
        </Card>
      ))}
    </div>
  );
}