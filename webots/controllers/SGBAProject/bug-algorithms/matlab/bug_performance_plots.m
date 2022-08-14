clear all, clc


load /home/knmcguire/Documents/experiments/bug_algorithms/results/results_03-30-2018_21-54.mat

bug_names = {'wf', 'com_bug', 'bug_2','alg_1', 'alg_2', 'i_bug','blind_bug'};

fitness = zeros(length(results.environment)-1,length(bug_names));
time = zeros(length(results.environment)-1,length(bug_names));
mean_gradient_env = zeros(length(bug_names),1);
mean_gradient = zeros(length(results.environment)-1,length(bug_names));

reached_goal = zeros(length(results.environment)-1,length(bug_names));
lenght_trajectory = zeros(length(results.environment)-1,length(bug_names));
lenght_trajectory_percentage = zeros(length(results.environment)-1,length(bug_names));

optimal_path_length_per_environment=0;
se = offsetstrel('ball',3,3);
for it = 1:length(results.environment)-1
    
    it
    img_dilated = results.environment(it).img;%imdilate(results.environment(it).img,se);

    did_all_bugs_make_it = 1;
    %   optimal_path_per_environment = astar_on_environment(results.environment(1).img,[9,9],[1,1]);
%     if(isfield(results.environment(it),'init_position'))
%         
%         optimal_path_per_environment = astar_on_environment(results.environment(it).img,results.environment(it).init_position(2,:)+7,results.environment(it).init_position(1,:)+7);
%     else
        optimal_path_per_environment = astar_on_environment(img_dilated,[2 2],[11 11]);
        
%     end
    diff_trajectory = diff(optimal_path_per_environment);
    
    
    
    if ~isempty(diff_trajectory)
        optimal_path_length_per_environment = sum(sqrt(diff_trajectory(:,1).^2+diff_trajectory(:,2).^2))/10;
        
        for itk = 1:length(results.environment(it).noise(1).bug)
            
            bug_temp = results.environment(it).noise(1).bug;
            index = find(strcmp({bug_temp.bug_name},bug_names{itk}));
            
            if(~isempty(index))
                fitness(it,itk) = results.environment(it).noise(1).bug(index).fitness(1);
                indices_time = find(results.environment(it).noise(1).bug(index).distances<1.5);
                
                
                %check of bugs toch bijna de goal hadden gehaald en
                %corriceer
                if(~isempty(indices_time))
                    results.environment(it).noise(1).bug(index).distances(indices_time(1):end)=[];
                    results.environment(it).noise(1).bug(index).trajectory(indices_time(1):end,:)=[];
                end
                [value_distance, min_index] = min(results.environment(it).noise(1).bug(index).distances);
                min_index = length(results.environment(it).noise(1).bug(index).distances);

                
                diff_trajectory = diff(results.environment(it).noise(1).bug(index).trajectory);
                
                
                
                temp_lenght_trajectory = sum(sqrt(diff_trajectory(1:min_index-1,1).^2+diff_trajectory(1:min_index-1,2).^2))+value_distance;
                lenght_trajectory(it,itk)= temp_lenght_trajectory;
                lenght_trajectory_percentage(it,itk)= temp_lenght_trajectory/optimal_path_length_per_environment;
                
                if(isempty(indices_time))
                    time(it,itk) = NaN;
                    %                     lenght_trajectory(it,:) = NaN;
                    %                     lenght_trajectory_percentage(it,:) = NaN;
                    reached_goal(it,itk) = 0;
                    did_all_bugs_make_it = 0;
                else
                    time(it,itk) = NaN;
                    reached_goal(it,itk) = 1;
                end
                
                %         temp_lenght_trajectory = 0;
                %         for itm = 1:size(results.environment(it).bug(index).trajectory,1)-1
                %             temp_lenght_trajectory = temp_lenght_trajectory + pdist(results.environment(it).bug(index).trajectory(itm:itm+1,:),'euclidean');
                %         end
                
                mean_gradient_env(itk) = mean(diff(results.environment(it).noise(1).bug(index).distances));
                mean_gradient(it,itk) =  mean(diff(results.environment(it).noise(1).bug(index).distances));
            end
            
        end
        if did_all_bugs_make_it ==false
            
            
        end
        
    else
        disp("A* produced an error")
        time(it,:) = NaN;
        lenght_trajectory(it,:) = NaN;
        lenght_trajectory_percentage(it,:) = NaN;
    end
    
    
end


%figure,boxplot(fitness, 'Labels',bug_names)
%figure,boxplot(time, 'Labels',bug_names)
%figure,boxplot(mean_gradient, 'Labels',bug_names)
%figure,boxplot(lenght_trajectory, 'Labels',bug_names)
figure,boxplot(100*lenght_trajectory_percentage(:,1:end-1), 'Labels',bug_names(1:end-1))
ylabel('Trajectory bug / Trajectory A*')
figure,bar(sum(reached_goal(:,1:end-1))/360)
set(gca, 'XTickLabel',bug_names(1:end-1),'DefaultTextInterpreter', 'none')

ylabel('bugs made to goal [%]')
%set(gca,'xticklabel',bug_names)

%%
% env_number = 1;
%     figure,
% 
%  for env_number = 1:359
%     env_number
%     for it = 1:length(results.environment(env_number).bug)
%         subplot(2,3,it),imshow(imresize(results.environment(env_number).img',2))
%         hold on
%         if(isfield(results.environment(env_number),'init_position'))
%             plot(20*(results.environment(env_number).init_position(:,2)+7),20*(results.environment(env_number).init_position(:,1)-1+7), 'o')
%         else
%             plot(20*[1 9],20*[1 9], 'o')
%         end
%         
%         
%         hold on, plot(20*(results.environment(env_number).bug(it).trajectory(2:end,2)+7),20*(results.environment(env_number).bug(it).trajectory(2:end,1)+7),'r')
%         
%         title([results.environment(env_number).bug(it).bug_name, ' (' num2str(size(results.environment(env_number).bug(it).trajectory,1)/10) ' sec)'],'Interpreter', 'none')
%     end
%     keyboard
%  end