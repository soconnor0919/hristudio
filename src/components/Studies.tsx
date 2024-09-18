"use client";

import React, { useState, useEffect } from 'react';
import { Card, CardContent, CardHeader, CardTitle } from "~/components/ui/card";
import { Button } from "~/components/ui/button";
import { Input } from "~/components/ui/input";
import { Dialog, DialogContent, DialogHeader, DialogTitle, DialogTrigger } from "~/components/ui/dialog";

interface Study {
  id: number;
  title: string;
  description: string;
}

export function Studies() {
  const [studies, setStudies] = useState<Study[]>([]);
  const [newStudy, setNewStudy] = useState({ title: '', description: '' });
  const [editingStudy, setEditingStudy] = useState<Study | null>(null);

  useEffect(() => {
    fetchStudies();
  }, []);

  const fetchStudies = async () => {
    const response = await fetch('/api/studies');
    const data = await response.json();
    setStudies(data);
  };

  const createStudy = async () => {
    const response = await fetch('/api/studies', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(newStudy),
    });
    const createdStudy = await response.json();
    setStudies([...studies, createdStudy]);
    setNewStudy({ title: '', description: '' });
  };

  const updateStudy = async () => {
    if (!editingStudy) return;
    const response = await fetch('/api/studies', {
      method: 'PUT',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(editingStudy),
    });
    const updatedStudy = await response.json();
    setStudies(studies.map(s => s.id === updatedStudy.id ? updatedStudy : s));
    setEditingStudy(null);
  };

  const deleteStudy = async (id: number) => {
    await fetch('/api/studies', {
      method: 'DELETE',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ id }),
    });
    setStudies(studies.filter(s => s.id !== id));
  };

  return (
    <div className="space-y-4">
      <h2 className="text-2xl font-bold">Studies</h2>
      <Dialog>
        <DialogTrigger asChild>
          <Button>Create New Study</Button>
        </DialogTrigger>
        <DialogContent>
          <DialogHeader>
            <DialogTitle>Create New Study</DialogTitle>
          </DialogHeader>
          <Input
            placeholder="Title"
            value={newStudy.title}
            onChange={(e) => setNewStudy({ ...newStudy, title: e.target.value })}
          />
          <Input
            placeholder="Description"
            value={newStudy.description}
            onChange={(e) => setNewStudy({ ...newStudy, description: e.target.value })}
          />
          <Button onClick={createStudy}>Create</Button>
        </DialogContent>
      </Dialog>
      {studies.map((study) => (
        <Card key={study.id}>
          <CardHeader>
            <CardTitle>{study.title}</CardTitle>
          </CardHeader>
          <CardContent>
            <p>{study.description}</p>
            <div className="flex space-x-2 mt-2">
              <Dialog>
                <DialogTrigger asChild>
                  <Button variant="outline" onClick={() => setEditingStudy(study)}>Edit</Button>
                </DialogTrigger>
                <DialogContent>
                  <DialogHeader>
                    <DialogTitle>Edit Study</DialogTitle>
                  </DialogHeader>
                  <Input
                    placeholder="Title"
                    value={editingStudy?.title || ''}
                    onChange={(e) => setEditingStudy({ ...editingStudy!, title: e.target.value })}
                  />
                  <Input
                    placeholder="Description"
                    value={editingStudy?.description || ''}
                    onChange={(e) => setEditingStudy({ ...editingStudy!, description: e.target.value })}
                  />
                  <Button onClick={updateStudy}>Update</Button>
                </DialogContent>
              </Dialog>
              <Button variant="destructive" onClick={() => deleteStudy(study.id)}>Delete</Button>
            </div>
          </CardContent>
        </Card>
      ))}
    </div>
  );
}