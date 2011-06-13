function [ ] = visualizeSkeleton( dataFile, frameNum );

fprintf('visualize skeleton!!\n');

fprintf(' skeleton data: %s\n', dataFile);
fprintf(' frame number:  %d\n\n', frameNum);

if ~exist(dataFile,'file'),
    error('FILE DOES NOT EXIST! CHECK FILE PATH AND FILE NAME!'); 
else
    figureTitle = strcat(dataFile, ' (frame:', int2str(frameNum),')');
    readSkel(dataFile,frameNum, figureTitle);
    fprintf('ALL DONE.\n');
end

end

