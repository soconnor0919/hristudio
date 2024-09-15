import { Card, CardContent, CardHeader, CardTitle, CardDescription } from '~/components/ui/card';
import { Button } from '~/components/ui/button';

const HomePage: React.FC = () => {
    return (
        <div className="min-h-screen bg-gradient-to-b from-blue-100 to-white pt-14 lg:pt-0">
            <div className="container mx-auto px-4 py-16">
                <header className="text-center mb-16">
                    <h1 className="text-5xl font-bold mb-4 text-blue-800">Welcome to the HRIStudio Dashboard!</h1>
                    <p className="text-xl text-gray-600 max-w-3xl mx-auto">
                        Manage your Human-Robot Interaction projects and experiments
                    </p>
                </header>

                <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-4">
                    <Card className="bg-white shadow-lg">
                        <CardHeader>
                            <CardTitle className="text-2xl font-semibold text-blue-700">Projects</CardTitle>
                            <CardDescription>Manage your HRI projects</CardDescription>
                        </CardHeader>
                        <CardContent>
                            <p className="mb-4">Create, edit, and analyze your HRI projects.</p>
                            <Button className="bg-blue-600 hover:bg-blue-700 text-white">View Projects</Button>
                        </CardContent>
                    </Card>

                    <Card className="bg-white shadow-lg">
                        <CardHeader>
                            <CardTitle className="text-2xl font-semibold text-blue-700">Experiments</CardTitle>
                            <CardDescription>Design and run experiments</CardDescription>
                        </CardHeader>
                        <CardContent>
                            <p className="mb-4">Set up, conduct, and analyze HRI experiments.</p>
                            <Button className="bg-green-600 hover:bg-green-700 text-white">New Experiment</Button>
                        </CardContent>
                    </Card>

                    <Card className="bg-white shadow-lg">
                        <CardHeader>
                            <CardTitle className="text-2xl font-semibold text-blue-700">Data Analysis</CardTitle>
                            <CardDescription>Analyze your research data</CardDescription>
                        </CardHeader>
                        <CardContent>
                            <p className="mb-4">Visualize and interpret your HRI research data.</p>
                            <Button className="bg-purple-600 hover:bg-purple-700 text-white">Analyze Data</Button>
                        </CardContent>
                    </Card>
                </div>
            </div>
        </div>
    );
};

export default HomePage;