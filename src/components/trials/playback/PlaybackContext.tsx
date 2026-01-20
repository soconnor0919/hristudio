"use client";

import React, { createContext, useContext, useEffect, useRef, useState } from "react";

interface TrialEvent {
    eventType: string;
    timestamp: Date;
    data?: unknown;
}

interface PlaybackContextType {
    // State
    currentTime: number;
    duration: number;
    isPlaying: boolean;
    playbackRate: number;
    startTime?: Date;

    // Actions
    play: () => void;
    pause: () => void;
    togglePlay: () => void;
    seekTo: (time: number) => void;
    setPlaybackRate: (rate: number) => void;
    setDuration: (duration: number) => void;
    setCurrentTime: (time: number) => void; // Used by VideoPlayer to update state

    // Data
    events: TrialEvent[];
    currentEventIndex: number; // Index of the last event that happened before currentTime
}

const PlaybackContext = createContext<PlaybackContextType | null>(null);

export function usePlayback() {
    const context = useContext(PlaybackContext);
    if (!context) {
        throw new Error("usePlayback must be used within a PlaybackProvider");
    }
    return context;
}

interface PlaybackProviderProps {
    children: React.ReactNode;
    events?: TrialEvent[];
    startTime?: Date;
}

export function PlaybackProvider({ children, events = [], startTime }: PlaybackProviderProps) {
    const [currentTime, setCurrentTime] = useState(0);
    const [duration, setDuration] = useState(0);
    const [isPlaying, setIsPlaying] = useState(false);
    const [playbackRate, setPlaybackRate] = useState(1);

    // Derived state: find the latest event index based on currentTime
    const currentEventIndex = React.useMemo(() => {
        if (!startTime || events.length === 0) return -1;

        // Find the last event that occurred before or at currentTime
        // Events are assumed to be sorted by timestamp
        // Using basic iteration for now, optimization possible for large lists
        let lastIndex = -1;

        for (let i = 0; i < events.length; i++) {
            const eventTime = new Date(events[i]!.timestamp).getTime();
            const startStr = new Date(startTime).getTime();
            const relativeSeconds = (eventTime - startStr) / 1000;

            if (relativeSeconds <= currentTime) {
                lastIndex = i;
            } else {
                break; // Events are sorted, so we can stop
            }
        }
        return lastIndex;
    }, [currentTime, events, startTime]);

    // Actions
    const play = () => setIsPlaying(true);
    const pause = () => setIsPlaying(false);
    const togglePlay = () => setIsPlaying(p => !p);

    const seekTo = (time: number) => {
        setCurrentTime(time);
        // Dispatch seek event to video player via some mechanism if needed, 
        // usually VideoPlayer observes this context or we use a Ref to control it.
        // Actually, simple way: Context holds state, VideoPlayer listens to state? 
        // No, VideoPlayer usually drives time. 
        // Let's assume VideoPlayer updates `setCurrentTime` as it plays.
        // But if *we* seek (e.g. from timeline), we need to tell VideoPlayer to jump.
        // We might need a `seekRequest` timestamp or similar signal.
    };

    const value: PlaybackContextType = {
        currentTime,
        duration,
        isPlaying,
        playbackRate,
        play,
        pause,
        togglePlay,
        seekTo,
        setPlaybackRate,
        setDuration,
        setCurrentTime,
        events,
        currentEventIndex,
    };

    return (
        <PlaybackContext.Provider value={value}>
            {children}
        </PlaybackContext.Provider>
    );
}
