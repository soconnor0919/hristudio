import { PERMISSIONS } from './permissions-client';

export const ROLES = {
  ADMIN: 'admin',
  PRINCIPAL_INVESTIGATOR: 'principal_investigator',
  RESEARCHER: 'researcher',
  WIZARD: 'wizard',
  OBSERVER: 'observer',
  ASSISTANT: 'assistant',
} as const;

export type RoleCode = keyof typeof ROLES;

export const ROLE_PERMISSIONS: Record<RoleCode, Array<keyof typeof PERMISSIONS>> = {
  ADMIN: Object.keys(PERMISSIONS) as Array<keyof typeof PERMISSIONS>,
  
  PRINCIPAL_INVESTIGATOR: [
    'CREATE_STUDY',
    'EDIT_STUDY',
    'DELETE_STUDY',
    'VIEW_STUDY',
    'VIEW_PARTICIPANT_NAMES',
    'CREATE_PARTICIPANT',
    'EDIT_PARTICIPANT',
    'DELETE_PARTICIPANT',
    'VIEW_ROBOT_STATUS',
    'VIEW_EXPERIMENT',
    'VIEW_EXPERIMENT_DATA',
    'EXPORT_EXPERIMENT_DATA',
    'ANNOTATE_EXPERIMENT',
    'MANAGE_ROLES',
    'MANAGE_USERS',
  ],
  
  RESEARCHER: [
    'VIEW_STUDY',
    'CREATE_PARTICIPANT',
    'EDIT_PARTICIPANT',
    'VIEW_ROBOT_STATUS',
    'VIEW_EXPERIMENT',
    'VIEW_EXPERIMENT_DATA',
    'EXPORT_EXPERIMENT_DATA',
    'ANNOTATE_EXPERIMENT',
  ],
  
  WIZARD: [
    'VIEW_STUDY',
    'VIEW_ROBOT_STATUS',
    'CONTROL_ROBOT',
    'RECORD_EXPERIMENT',
    'VIEW_EXPERIMENT',
    'ANNOTATE_EXPERIMENT',
  ],
  
  OBSERVER: [
    'VIEW_STUDY',
    'VIEW_ROBOT_STATUS',
    'VIEW_EXPERIMENT',
    'VIEW_EXPERIMENT_DATA',
    'ANNOTATE_EXPERIMENT',
  ],
  
  ASSISTANT: [
    'VIEW_STUDY',
    'VIEW_ROBOT_STATUS',
    'VIEW_EXPERIMENT',
  ],
}; 