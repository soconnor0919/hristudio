import { Card, CardContent, CardHeader, CardTitle, CardDescription } from '~/components/ui/card';
import { Button } from '~/components/ui/button';
import { Studies } from "~/components/Studies";

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
                <Studies />
            </div>
        </div>
    );
};

export default HomePage;