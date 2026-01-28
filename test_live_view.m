%% Test Edsdk matlab live view
% ------------READ:-----------------------------------
% Must initialize edsdk first and startLiveView before starting timer from
% the command window.
% must stop live view and terminate when done.
% -------------------------------------------------------

clc; clear; close all;

f = figure();
ax1 = axes('Parent',f);

picture = image(zeros(640,960,3),'Parent',ax1);

t = timer("BusyMode","drop","ExecutionMode",'fixedSpacing','Period',1/30,TimerFcn=@updateFrame);

t.UserData.f = f;
t.UserData.ax = ax1;
t.UserData.picture = picture;

function updateFrame(obj,event)
    frame = edsdk_mex('getFrame');

    obj.UserData.picture.CData = frame;
    %set(UserData.picture,'CData',frame);
    drawnow limitrate nocallbacks
end