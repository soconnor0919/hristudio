export const PERMISSIONS = {
  // Study permissions
  CREATE_STUDY: "create_study",
  EDIT_STUDY: "edit_study",
  DELETE_STUDY: "delete_study",
  VIEW_STUDY: "view_study",
  
  // Participant permissions
  VIEW_PARTICIPANT_NAMES: "view_participant_names",
  CREATE_PARTICIPANT: "create_participant",
  EDIT_PARTICIPANT: "edit_participant",
  DELETE_PARTICIPANT: "delete_participant",
  
  // Robot permissions
  CONTROL_ROBOT: "control_robot",
  VIEW_ROBOT_STATUS: "view_robot_status",
  
  // Experiment permissions
  RECORD_EXPERIMENT: "record_experiment",
  VIEW_EXPERIMENT: "view_experiment",
  VIEW_EXPERIMENT_DATA: "view_experiment_data",
  EXPORT_EXPERIMENT_DATA: "export_experiment_data",
  ANNOTATE_EXPERIMENT: "annotate_experiment",
  
  // Administrative permissions
  MANAGE_ROLES: "manage_roles",
  MANAGE_USERS: "manage_users",
  MANAGE_SYSTEM_SETTINGS: "manage_system_settings",
} as const;

export type Permission = keyof typeof PERMISSIONS;
export type PermissionValue = (typeof PERMISSIONS)[Permission];

export const ROLES = {
  ADMIN: "admin",
  PRINCIPAL_INVESTIGATOR: "principal_investigator",
  RESEARCHER: "researcher",
  WIZARD: "wizard",
  OBSERVER: "observer",
  ASSISTANT: "assistant",
} as const;

export type Role = keyof typeof ROLES;
export type RoleValue = (typeof ROLES)[Role];

export const ROLE_PERMISSIONS: Record<Role, Permission[]> = {
  ADMIN: Object.keys(PERMISSIONS) as Permission[],
  
  PRINCIPAL_INVESTIGATOR: [
    "CREATE_STUDY",
    "EDIT_STUDY",
    "DELETE_STUDY",
    "VIEW_STUDY",
    "VIEW_PARTICIPANT_NAMES",
    "CREATE_PARTICIPANT",
    "EDIT_PARTICIPANT",
    "DELETE_PARTICIPANT",
    "VIEW_ROBOT_STATUS",
    "VIEW_EXPERIMENT",
    "VIEW_EXPERIMENT_DATA",
    "EXPORT_EXPERIMENT_DATA",
    "ANNOTATE_EXPERIMENT",
    "MANAGE_ROLES",
    "MANAGE_USERS",
  ],
  
  RESEARCHER: [
    "VIEW_STUDY",
    "VIEW_ROBOT_STATUS",
    "VIEW_EXPERIMENT",
    "VIEW_EXPERIMENT_DATA",
    "EXPORT_EXPERIMENT_DATA",
    "ANNOTATE_EXPERIMENT",
  ],
  
  WIZARD: [
    "VIEW_STUDY",
    "VIEW_ROBOT_STATUS",
    "CONTROL_ROBOT",
    "RECORD_EXPERIMENT",
    "VIEW_EXPERIMENT",
    "ANNOTATE_EXPERIMENT",
  ],
  
  OBSERVER: [
    "VIEW_STUDY",
    "VIEW_ROBOT_STATUS",
    "VIEW_EXPERIMENT",
    "VIEW_EXPERIMENT_DATA",
    "ANNOTATE_EXPERIMENT",
  ],
  
  ASSISTANT: [
    "VIEW_STUDY",
    "VIEW_ROBOT_STATUS",
    "VIEW_EXPERIMENT",
  ],
};

export const ROLE_DESCRIPTIONS: Record<Role, string> = {
  ADMIN: "Full system administrator with all permissions",
  PRINCIPAL_INVESTIGATOR: "Lead researcher responsible for study design and oversight",
  RESEARCHER: "Study team member with access to anonymized data and experiment monitoring capabilities",
  WIZARD: "Operator controlling robot behavior during experiments",
  OBSERVER: "Team member observing and annotating experiments",
  ASSISTANT: "Support staff with limited view access",
};

export const PERMISSION_DESCRIPTIONS: Record<Permission, string> = {
  CREATE_STUDY: "Create new research studies",
  EDIT_STUDY: "Modify existing study parameters",
  DELETE_STUDY: "Remove studies from the system",
  VIEW_STUDY: "View study details and progress",
  VIEW_PARTICIPANT_NAMES: "Access participant identifying information",
  CREATE_PARTICIPANT: "Add new participants to studies",
  EDIT_PARTICIPANT: "Update participant information",
  DELETE_PARTICIPANT: "Remove participants from studies",
  CONTROL_ROBOT: "Operate robot during experiments",
  VIEW_ROBOT_STATUS: "Monitor robot state and sensors",
  RECORD_EXPERIMENT: "Start/stop experiment recording",
  VIEW_EXPERIMENT: "View experiment progress and details",
  VIEW_EXPERIMENT_DATA: "Access collected experiment data",
  EXPORT_EXPERIMENT_DATA: "Download experiment data",
  ANNOTATE_EXPERIMENT: "Add notes and annotations to experiments",
  MANAGE_ROLES: "Assign and modify user roles",
  MANAGE_USERS: "Add and remove system users",
  MANAGE_SYSTEM_SETTINGS: "Configure system-wide settings",
}; 