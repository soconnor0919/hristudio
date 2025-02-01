"use client";

import { Dialog, DialogContent, DialogTitle } from "./dialog";
import { useState, useCallback } from "react";
import Cropper, { Area } from "react-easy-crop";
import { Button } from "./button";
import { cn } from "~/lib/utils";
import { Slider } from "./slider";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "./card";
import { Loader2 } from "lucide-react";

interface ImageCropModalProps {
  file: File;
  aspect?: number;
  onCrop: (blob: Blob) => void;
  onCancel: () => void;
  className?: string;
  cropBoxClassName?: string;
  overlayClassName?: string;
}

export function ImageCropModal({
  file,
  aspect = 1,
  onCrop,
  onCancel,
  className,
  cropBoxClassName,
  overlayClassName,
}: ImageCropModalProps) {
  const [crop, setCrop] = useState({ x: 0, y: 0 });
  const [zoom, setZoom] = useState(1.5);
  const [croppedAreaPixels, setCroppedAreaPixels] = useState<Area | null>(null);
  const [imageUrl] = useState(() => URL.createObjectURL(file));
  const [isCropping, setIsCropping] = useState(false);

  const onCropComplete = useCallback((croppedArea: Area, croppedAreaPixels: Area) => {
    setCroppedAreaPixels(croppedAreaPixels);
  }, []);

  const handleCrop = async () => {
    try {
      setIsCropping(true);
      if (!croppedAreaPixels) return;

      const canvas = document.createElement("canvas");
      const image = new Image();
      image.src = imageUrl;

      await new Promise((resolve) => {
        image.onload = resolve;
      });

      const scaleX = image.naturalWidth / image.width;
      const scaleY = image.naturalHeight / image.height;

      // Set a fixed size for the output
      const outputSize = 400;
      canvas.width = outputSize;
      canvas.height = outputSize;

      const ctx = canvas.getContext("2d");
      if (!ctx) return;

      ctx.imageSmoothingQuality = "high";
      ctx.drawImage(
        image,
        croppedAreaPixels.x * scaleX,
        croppedAreaPixels.y * scaleY,
        croppedAreaPixels.width * scaleX,
        croppedAreaPixels.height * scaleY,
        0,
        0,
        outputSize,
        outputSize
      );

      canvas.toBlob(
        (blob) => {
          if (blob) {
            onCrop(blob);
          }
        },
        "image/jpeg",
        0.95
      );
    } catch (error) {
      console.error("Error cropping image:", error);
    } finally {
      setIsCropping(false);
    }
  };

  return (
    <Dialog open onOpenChange={() => onCancel()}>
      <DialogContent className={cn("max-h-[85vh] gap-0 p-0 sm:max-w-md", className)}>
        <div className="space-y-4 p-6 pb-4">
          <DialogTitle className="text-lg font-semibold">
            Crop Profile Picture
          </DialogTitle>
          <p className="text-sm text-muted-foreground">
            Adjust the image and zoom to crop your profile picture
          </p>
        </div>
        <div className="space-y-6 px-6">
          <div className="relative h-[280px] w-full overflow-hidden rounded-lg">
            <Cropper
              image={imageUrl}
              crop={crop}
              zoom={zoom}
              aspect={aspect}
              onCropChange={setCrop}
              onCropComplete={onCropComplete}
              onZoomChange={setZoom}
              cropShape="round"
              showGrid={false}
              objectFit="contain"
              minZoom={1}
              maxZoom={3}
              classes={{
                containerClassName: "rounded-lg",
                cropAreaClassName: cn(
                  "!border-2 !border-primary !rounded-full !shadow-2xl",
                  cropBoxClassName
                ),
                mediaClassName: "object-contain",
              }}
              style={{
                containerStyle: {
                  position: "relative",
                  width: "100%",
                  height: "100%",
                  backgroundColor: "var(--muted)",
                },
                cropAreaStyle: {
                  color: "var(--muted-foreground)",
                },
              }}
            />
          </div>
          <div className="space-y-2">
            <h4 className="font-medium leading-none">Zoom</h4>
            <Slider
              value={[zoom]}
              min={1}
              max={3}
              step={0.1}
              onValueChange={([value]) => setZoom(value)}
              className="py-2"
              aria-label="Zoom"
            />
          </div>
          <div className="flex justify-end gap-2 pb-6">
            <Button variant="outline" onClick={onCancel}>
              Cancel
            </Button>
            <Button onClick={handleCrop} disabled={isCropping}>
              {isCropping && (
                <Loader2 className="mr-2 size-4 animate-spin" />
              )}
              Save & Apply
            </Button>
          </div>
        </div>
      </DialogContent>
    </Dialog>
  );
} 
