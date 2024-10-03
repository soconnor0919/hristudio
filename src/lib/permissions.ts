import { auth } from "@clerk/nextjs/server";
import { Participant } from "../types/Participant";

export const isUserAuthorized = (userId: string | null): boolean => {
  // Implement your logic to determine if the user is authorized to see participant names
  // For example, you might check if the user is an admin or has a specific role
//   return userId !== null; // Placeholder logic, replace with your actual authorization logic
    return false;
};

export const anonymizeParticipants = (participants: Participant[], userId: string | null): Participant[] => {
  if (isUserAuthorized(userId)) {
    return participants; // Return original participants if authorized
  }

  return participants.map(participant => ({
    ...participant,
    name: `Participant ${participant.id}`, // Anonymize the name
  }));
};
