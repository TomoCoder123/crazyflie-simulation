close all, clear all, clc
%  folderpath = '/home/knmcguire/Documents/Software/catkin_ws/src'
%  rosgenmsg(folderpath);

rosinit
with_plot = false;
bug_names = {'wf', 'com_bug', 'bug_2','alg_1', 'alg_2', 'i_bug', 'blind_bug'};
switch_bug_pub = rospublisher('/switch_bug','std_msgs/String');
environment_random_pub = rospublisher('/random_environment','std_msgs/Bool');
noise_level_pub = rospublisher('/noise_level','std_msgs/Float64');


msg = rosmessage(switch_bug_pub);
msg_env = rosmessage(environment_random_pub);
msg_noise = rosmessage(noise_level_pub);

amount_of_bugs = 6;
amount_of_environments = 1;
%noise_levels = [0.05 0.1 0.15, 0.2];
% noise_levels = [0:0.2:1.0];
%noise_levels = [0.0:1:6.0];
%noise_levels = [0.0:0.2:1.0];

noise_levels=0.0;
for itk = 1:amount_of_environments
    disp(itk)
    msg_env.Data = 1;
    send(environment_random_pub,msg_env);
    pause(2)
    
    for itn = 1:length(noise_levels)
        msg_noise.Data = noise_levels(itn);
        send(noise_level_pub,msg_noise);
        pause(2)
        
        for it = 1:6
            
            msg.Data = bug_names{it};
            send(switch_bug_pub,msg);
            disp("Send out switch bug message")
            
            pause(2)
            sub = rossubscriber('/finished_sim_matlab');
            receive(sub);
            
            disp("Simulation is Finished")
            pause(5)
            
            results.environment(itk).noise(itn).bug(it).bug_name = bug_names{it};
            results.environment(itk).noise(itn).bug(it).trajectory = csvread("/home/knmcguire/.ros/trajectory.txt");
            results.environment(itk).noise(itn).bug(it).fitness = csvread("/home/knmcguire/.ros/fitness.txt");
            results.environment(itk).noise(itn).bug(it).distances = csvread("/home/knmcguire/.ros/distances.txt");
            results.environment(itk).noise(itn).bug(it).trajectory = csvread("/home/knmcguire/.ros/trajectory.txt");
            
        end
        results.environment(itk).img= imread('/home/knmcguire/.ros/environment.png');
        % results.environment(itk).init_position= csvread('/home/knmcguire/.ros/init_position.txt');
        if with_plot
            if itk < 9
                figure(1)
                
                subplot(3,3,itk),imshow(imresize(results.environment(itk).img',2))
                for it = 1:amount_of_bugs
                    hold on, plot(20*(results.environment(itk).noise(itn).bug(it).trajectory(:,2)+10),20*(results.environment(itk).noise(itn).bug(it).trajectory(:,1)+10))
                end
                
            end
        end
    end
    
    
end
if with_plot
Filename = sprintf('trajectories_%s.png', datestr(now,'mm-dd-yyyy_HH-MM'));
saveas(gcf,Filename)
end
Filename = sprintf('results_%s.mat', datestr(now,'mm-dd-yyyy_HH-MM'));
save(Filename, 'results')

rosshutdown