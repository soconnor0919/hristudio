"use client";

import React, { useRef, useEffect } from "react";
import { usePlayback } from "./PlaybackContext";
import { AspectRatio } from "~/components/ui/aspect-ratio";
import { Loader2, Play, Pause, Volume2, VolumeX, Maximize } from "lucide-react";
import { Slider } from "~/components/ui/slider";
import { Button } from "~/components/ui/button";

interface PlaybackPlayerProps {
    src: string;
}

export function PlaybackPlayer({ src }: PlaybackPlayerProps) {
    const videoRef = useRef<HTMLVideoElement>(null);
    const {
        currentTime,
        isPlaying,
        playbackRate,
        setCurrentTime,
        setDuration,
        togglePlay,
        play,
        pause
    } = usePlayback();

    const [isBuffering, setIsBuffering] = React.useState(true);
    const [volume, setVolume] = React.useState(1);
    const [muted, setMuted] = React.useState(false);

    // Sync Play/Pause state
    useEffect(() => {
        const video = videoRef.current;
        if (!video) return;

        if (isPlaying && video.paused) {
            video.play().catch(console.error);
        } else if (!isPlaying && !video.paused) {
            video.pause();
        }
    }, [isPlaying]);

    // Sync Playback Rate
    useEffect(() => {
        if (videoRef.current) {
            videoRef.current.playbackRate = playbackRate;
        }
    }, [playbackRate]);

    // Sync Seek (External seek request)
    // Note: This is tricky because normal playback also updates currentTime.
    // We need to differentiate between "time updated by video" and "time updated by user seek".
    // For now, we'll let the video drive the context time, and rely on the Parent/Context
    // to call a imperative sync if needed, or we implement a "seekRequest" signal in context.
    // simpler: If context time differs significantly from video time, we seek.
    useEffect(() => {
        const video = videoRef.current;
        if (!video) return;

        if (Math.abs(video.currentTime - currentTime) > 0.5) {
            video.currentTime = currentTime;
        }
    }, [currentTime]);

    const handleTimeUpdate = () => {
        if (videoRef.current) {
            setCurrentTime(videoRef.current.currentTime);
        }
    };

    const handleLoadedMetadata = () => {
        if (videoRef.current) {
            setDuration(videoRef.current.duration);
            setIsBuffering(false);
        }
    };

    const handleWaiting = () => setIsBuffering(true);
    const handlePlaying = () => setIsBuffering(false);
    const handleEnded = () => pause();

    return (
        <div className="group relative rounded-lg overflow-hidden border bg-black shadow-sm">
            <AspectRatio ratio={16 / 9}>
                <video
                    ref={videoRef}
                    src={src}
                    className="w-full h-full object-contain"
                    onTimeUpdate={handleTimeUpdate}
                    onLoadedMetadata={handleLoadedMetadata}
                    onWaiting={handleWaiting}
                    onPlaying={handlePlaying}
                    onEnded={handleEnded}
                    onClick={togglePlay}
                />

                {/* Overlay Controls (Visible on Hover/Pause) */}
                <div className="absolute inset-x-0 bottom-0 bg-gradient-to-t from-black/80 to-transparent p-4 opacity-0 transition-opacity group-hover:opacity-100 data-[paused=true]:opacity-100" data-paused={!isPlaying}>
                    <div className="flex items-center gap-4">
                        <Button
                            variant="ghost"
                            size="icon"
                            className="text-white hover:bg-white/20"
                            onClick={togglePlay}
                        >
                            {isPlaying ? <Pause className="h-6 w-6" /> : <Play className="h-6 w-6 fill-current" />}
                        </Button>

                        <div className="flex-1">
                            <Slider
                                value={[currentTime]}
                                min={0}
                                max={videoRef.current?.duration || 100}
                                step={0.1}
                                onValueChange={([val]) => {
                                    if (videoRef.current) {
                                        videoRef.current.currentTime = val;
                                        setCurrentTime(val);
                                    }
                                }}
                                className="cursor-pointer"
                            />
                        </div>

                        <div className="text-xs font-mono text-white/90">
                            {formatTime(currentTime)} / {formatTime(videoRef.current?.duration || 0)}
                        </div>

                        <Button
                            variant="ghost"
                            size="icon"
                            className="text-white hover:bg-white/20"
                            onClick={() => setMuted(!muted)}
                        >
                            {muted || volume === 0 ? <VolumeX className="h-5 w-5" /> : <Volume2 className="h-5 w-5" />}
                        </Button>
                    </div>
                </div>

                {isBuffering && (
                    <div className="absolute inset-0 flex items-center justify-center bg-black/20 pointer-events-none">
                        <Loader2 className="h-10 w-10 animate-spin text-white/80" />
                    </div>
                )}
            </AspectRatio>
        </div>
    );
}

function formatTime(seconds: number) {
    const m = Math.floor(seconds / 60);
    const s = Math.floor(seconds % 60);
    return `${m}:${s.toString().padStart(2, "0")}`;
}
