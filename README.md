# HRIStudio

HRIStudio is a web-based platform designed to streamline the design, execution, and analysis of Wizard-of-Oz (WoZ) experiments in Human-Robot Interaction (HRI) studies. It offers an intuitive interface for experiment creation, real-time control and monitoring during experimental runs, and comprehensive data logging and playback tools for analysis and reproducibility.

## Features

- User-friendly graphical interface for designing WoZ experiments
- Visual programming environment with drag-and-drop functionality
- Real-time control and observation capabilities during live experiment sessions
- Comprehensive data logging and playback tools
- Integration with Robot Operating System (ROS) for various robotic platforms
- Collaborative workflow support with multiple user accounts and data sharing

## System Requirements

- Node.js (version X.X.X or higher)
- npm (version X.X.X or higher)
- ROS (Robot Operating System)

## Installation

1. Clone the repository:
   ```
   git clone https://github.com/your-username/hristudio.git
   ```

2. Navigate to the project directory:
   ```
   cd hristudio
   ```

3. Install dependencies:
   ```
   npm install
   ```

4. Set up environment variables:
   Create a `.env.local` file in the root directory and add the necessary environment variables (e.g., database connection string, API keys).

5. Run the development server:
   ```
   npm run dev
   ```

6. Open [http://localhost:3000](http://localhost:3000) in your browser to see the application.

## Project Structure

- `pages/`: Contains the Next.js pages
- `components/`: Reusable React components
- `public/`: Static assets
- `styles/`: CSS styles
- `lib/`: Utility functions and modules
- `api/`: API routes
- `ros/`: ROS interface and related components

## Usage

1. Create a new study in the Dashboard view
2. Design your experiment using the visual programming interface in the Design view
3. Execute the experiment using the Execute view
4. Analyze results and playback recorded data in the Playback view

For detailed usage instructions, please refer to the [User Guide](link-to-user-guide).

## Contributing

We welcome contributions to HRIStudio! Please read our [Contributing Guidelines](link-to-contributing-guidelines) for more information on how to get started.

## License

This project is licensed under the [MIT License](link-to-license-file).

## Contact

For questions or support, please contact [your-email@example.com](mailto:your-email@example.com).

## Acknowledgments

This project is being developed by Sean O'Connor and L. Felipe Perrone at Bucknell University. We would like to thank the robotics and HRI research community for their valuable insights and contributions.