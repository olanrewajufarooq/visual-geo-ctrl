function writeReplayArtifact(outputPath, traj)
%WRITEREPLAYARTIFACT Save one processed replay trajectory.
%   Keeping SAVE in this ordinary function avoids parfor workspace
%   transparency violations caused by dynamic workspace saves.

save(outputPath, 'traj');
end
