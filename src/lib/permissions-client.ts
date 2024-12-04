export const PERMISSIONS = {
  CREATE_STUDY: 'create_study',
  EDIT_STUDY: 'edit_study',
  DELETE_STUDY: 'delete_study',
  VIEW_STUDY: 'view_study',
  VIEW_PARTICIPANT_NAMES: 'view_participant_names',
  CREATE_PARTICIPANT: 'create_participant',
  EDIT_PARTICIPANT: 'edit_participant',
  DELETE_PARTICIPANT: 'delete_participant',
  CONTROL_ROBOT: 'control_robot',
  VIEW_ROBOT_STATUS: 'view_robot_status',
  RECORD_EXPERIMENT: 'record_experiment',
  VIEW_EXPERIMENT: 'view_experiment',
  VIEW_EXPERIMENT_DATA: 'view_experiment_data',
  EXPORT_EXPERIMENT_DATA: 'export_experiment_data',
  ANNOTATE_EXPERIMENT: 'annotate_experiment',
  MANAGE_ROLES: 'manage_roles',
  MANAGE_USERS: 'manage_users',
  MANAGE_SYSTEM_SETTINGS: 'manage_system_settings',
} as const;

export function hasPermission(permissions: string[], permission: string): boolean {
  return permissions.includes(permission);
} 