export const PERMISSIONS = {
  // Study Management
  CREATE_STUDY: "create_study",
  EDIT_STUDY: "edit_study",
  DELETE_STUDY: "delete_study",
  TRANSFER_OWNERSHIP: "transfer_ownership",
  VIEW_STUDY: "view_study",
  MANAGE_STUDY_METADATA: "manage_study_metadata",
  
  // Participant Management
  ADD_PARTICIPANT: "add_participant",
  EDIT_PARTICIPANT: "edit_participant",
  DELETE_PARTICIPANT: "delete_participant",
  VIEW_PARTICIPANT_IDENTIFIABLE: "view_participant_identifiable",
  VIEW_PARTICIPANT_ANONYMIZED: "view_participant_anonymized",
  MANAGE_CONSENT_FORMS: "manage_consent_forms",
  
  // Experiment Design
  CREATE_EXPERIMENT: "create_experiment",
  EDIT_EXPERIMENT: "edit_experiment",
  DELETE_EXPERIMENT: "delete_experiment",
  DEFINE_ROBOT_BEHAVIORS: "define_robot_behaviors",
  CONFIGURE_DATA_COLLECTION: "configure_data_collection",
  
  // Experiment Execution
  RUN_TRIALS: "run_trials",
  CONTROL_ROBOT: "control_robot",
  MONITOR_SESSIONS: "monitor_sessions",
  ADD_REALTIME_ANNOTATIONS: "add_realtime_annotations",
  
  // Data Access
  VIEW_RAW_DATA: "view_raw_data",
  VIEW_ANONYMIZED_DATA: "view_anonymized_data",
  EXPORT_DATA: "export_data",
  
  // User Management
  INVITE_USERS: "invite_users",
  ASSIGN_ROLES: "assign_roles",
  REMOVE_USERS: "remove_users",
  MANAGE_PERMISSIONS: "manage_permissions",
} as const;

export type Permission = keyof typeof PERMISSIONS;
export type PermissionValue = (typeof PERMISSIONS)[Permission];

export const ROLES = {
  OWNER: "owner",
  ADMIN: "admin",
  PRINCIPAL_INVESTIGATOR: "principal_investigator",
  WIZARD: "wizard",
  RESEARCHER: "researcher",
  OBSERVER: "observer",
} as const;

export type Role = keyof typeof ROLES;
export type RoleValue = (typeof ROLES)[Role];

export const ROLE_PERMISSIONS: Record<Role, Permission[]> = {
  OWNER: Object.keys(PERMISSIONS) as Permission[],
  
  ADMIN: [
    "EDIT_STUDY",
    "VIEW_STUDY",
    "MANAGE_STUDY_METADATA",
    "ADD_PARTICIPANT",
    "EDIT_PARTICIPANT",
    "DELETE_PARTICIPANT",
    "VIEW_PARTICIPANT_IDENTIFIABLE",
    "VIEW_PARTICIPANT_ANONYMIZED",
    "MANAGE_CONSENT_FORMS",
    "CREATE_EXPERIMENT",
    "EDIT_EXPERIMENT",
    "DELETE_EXPERIMENT",
    "DEFINE_ROBOT_BEHAVIORS",
    "CONFIGURE_DATA_COLLECTION",
    "RUN_TRIALS",
    "CONTROL_ROBOT",
    "MONITOR_SESSIONS",
    "ADD_REALTIME_ANNOTATIONS",
    "VIEW_RAW_DATA",
    "VIEW_ANONYMIZED_DATA",
    "EXPORT_DATA",
    "INVITE_USERS",
    "ASSIGN_ROLES",
    "REMOVE_USERS",
  ],
  
  PRINCIPAL_INVESTIGATOR: [
    "VIEW_STUDY",
    "ADD_PARTICIPANT",
    "EDIT_PARTICIPANT",
    "DELETE_PARTICIPANT",
    "VIEW_PARTICIPANT_IDENTIFIABLE",
    "VIEW_PARTICIPANT_ANONYMIZED",
    "MANAGE_CONSENT_FORMS",
    "CREATE_EXPERIMENT",
    "EDIT_EXPERIMENT",
    "DELETE_EXPERIMENT",
    "DEFINE_ROBOT_BEHAVIORS",
    "CONFIGURE_DATA_COLLECTION",
    "RUN_TRIALS",
    "CONTROL_ROBOT",
    "MONITOR_SESSIONS",
    "ADD_REALTIME_ANNOTATIONS",
    "VIEW_RAW_DATA",
    "VIEW_ANONYMIZED_DATA",
    "EXPORT_DATA",
    "INVITE_USERS",
  ],
  
  WIZARD: [
    "VIEW_STUDY",
    "VIEW_PARTICIPANT_ANONYMIZED",
    "RUN_TRIALS",
    "CONTROL_ROBOT",
    "MONITOR_SESSIONS",
    "ADD_REALTIME_ANNOTATIONS",
    "VIEW_ANONYMIZED_DATA",
  ],
  
  RESEARCHER: [
    "VIEW_STUDY",
    "VIEW_PARTICIPANT_ANONYMIZED",
    "MONITOR_SESSIONS",
    "ADD_REALTIME_ANNOTATIONS",
    "VIEW_ANONYMIZED_DATA",
    "EXPORT_DATA",
  ],
  
  OBSERVER: [
    "VIEW_STUDY",
    "VIEW_PARTICIPANT_ANONYMIZED",
    "MONITOR_SESSIONS",
    "ADD_REALTIME_ANNOTATIONS",
  ],
};

export const ROLE_DESCRIPTIONS: Record<Role, string> = {
  OWNER: "Study owner with full control and exclusive ability to delete study or transfer ownership",
  ADMIN: "Administrator with ability to manage participants, experiments, and other members",
  PRINCIPAL_INVESTIGATOR: "Scientific lead with full access to participant data and experiment design",
  WIZARD: "Operator controlling robot behavior during experiment trials",
  RESEARCHER: "Team member who can analyze experiment data and results",
  OBSERVER: "Team member who can view experiments and add annotations",
};

export const PERMISSION_DESCRIPTIONS: Record<Permission, string> = {
  // Study Management
  CREATE_STUDY: "Create new research studies",
  EDIT_STUDY: "Modify existing study parameters",
  DELETE_STUDY: "Remove studies from the system",
  TRANSFER_OWNERSHIP: "Transfer study ownership to another user",
  VIEW_STUDY: "View study details and progress",
  MANAGE_STUDY_METADATA: "Manage study metadata and settings",
  
  // Participant Management
  ADD_PARTICIPANT: "Add new participants to studies",
  EDIT_PARTICIPANT: "Update participant information",
  DELETE_PARTICIPANT: "Remove participants from studies",
  VIEW_PARTICIPANT_IDENTIFIABLE: "Access participant identifying information",
  VIEW_PARTICIPANT_ANONYMIZED: "View anonymized participant data",
  MANAGE_CONSENT_FORMS: "Manage participant consent forms",
  
  // Experiment Design
  CREATE_EXPERIMENT: "Create new experiments",
  EDIT_EXPERIMENT: "Modify existing experiments",
  DELETE_EXPERIMENT: "Remove experiments from studies",
  DEFINE_ROBOT_BEHAVIORS: "Define and configure robot behaviors",
  CONFIGURE_DATA_COLLECTION: "Configure experiment data collection",
  
  // Experiment Execution
  RUN_TRIALS: "Execute experiment trials",
  CONTROL_ROBOT: "Control robot during experiments",
  MONITOR_SESSIONS: "Monitor live experiment sessions",
  ADD_REALTIME_ANNOTATIONS: "Add annotations during experiments",
  
  // Data Access
  VIEW_RAW_DATA: "Access raw experiment data",
  VIEW_ANONYMIZED_DATA: "Access anonymized experiment data",
  EXPORT_DATA: "Export experiment data",
  
  // User Management
  INVITE_USERS: "Invite new users to the study",
  ASSIGN_ROLES: "Assign roles to study members",
  REMOVE_USERS: "Remove users from the study",
  MANAGE_PERMISSIONS: "Manage user permissions",
}; 