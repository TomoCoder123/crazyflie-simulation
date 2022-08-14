clear all;
close all;
clc;
dirname='/home/knmcguire/Software/catkin_ws/src/bug_algorithms/statistics_scripts/experiments';
% testname = strcat(dirname,'/test_20181024_191634');
%    testname = strcat(dirname,'/test_20181018_183757');
%testname = strcat(dirname,'/test_20181021_150510');
% testname = strcat(dirname,'/test_20190116_200134');
% testname = strcat(dirname,'/test_20190117_213842'); % 1 agent
% testname = strcat(dirname,'/test_20190118_093311'); % 3 agents

testname = strcat(dirname,'/test_20190119_091420'); % 5 agents
testname = strcat(dirname,'/test_20190120_121046'); % 8 agents
testname = strcat(dirname,'/test_20190122_114700'); % 2 agents
testname = strcat(dirname,'/test_20190123_084632'); % 4 agents
testname = strcat(dirname,'/test_20190124_084420'); % 6 agents

testname = strcat(dirname,'/test_20190130_232402'); % 6 agents
testname = strcat(dirname,'/test_20190131_165220'); % 6 agents
testname = strcat(dirname,'/test_20190201_215121'); % 6 agents
testname = strcat(dirname,'/test_20190203_120405'); % 2 agents

testname = strcat(dirname,'/test_20190204_153502'); % 2 agents
testname = strcat(dirname,'/test_20190204_225154'); % 6 agents
testname = strcat(dirname,'/test_20190210_154859'); % 8 agents
testname = strcat(dirname,'/test_20190211_203758'); % 4 agents
testname = strcat(dirname,'/test_20190213_091903'); % 10 agents


testname = strcat(dirname,'/test_20190217_230948'); % 2 agents
% testname = strcat(dirname,'/test_20190218_101020'); % 6 agents


amount_of_agents = 2;
amount_of_environments = 48


do_plot = false;

time_treshold = 3900:2500:28900;
time_treshold = 9000 %9000;%28900;
size_threshold = size(time_treshold,2);
map = jet(size_threshold);
id_color_map = [1 0 0; 0 1 0; 0 0 1; 0.5 0.5 0; 0 0.5 0.5; 0.3 0.3 0.3; 0.5 0 0; 0 0.5 0; 0 0 0.5];

coverage = [];

for itt = 1:size_threshold
    
    for(it=0:amount_of_environments)
        occMap = robotics.BinaryOccupancyGrid(20,20,1);
        occMap_return = robotics.BinaryOccupancyGrid(20,20,1);
        
        filename = strcat(testname,'/environment',num2str(it));
        
        img=255-imread(strcat(filename,'/environment.png'));
        
        if do_plot
            figure(1),
            hold on,
            xImg = linspace(-10, 10, size(img, 2));
            yImg = linspace(-10,10, size(img, 1));
            image(xImg, yImg, img, 'CDataMapping', 'scaled');
            colormap(gray(256))
        end
        
        
        accessable_area_percentage =1;
%         %calculate which part of the environment is accessable
%         filename = strcat(testname,'/environment',num2str(it));
%         img=255-imread(strcat(filename,'/environment.png'));
%         
%         img_bin = im2bw(img);
%         CC=bwconncomp(img_bin);
%         
%         linearInd = sub2ind([200,200],100,100);
%         index = 0;
%         for it_temp=1:size(CC.PixelIdxList,1)
%             if ismember(linearInd,CC.PixelIdxList{it_temp})
%                 index=it_temp;
%                 break
%             end
%         end
%         accessable_area_percentage = size(CC.PixelIdxList{index},1)/(200*200);
        
        
        for itn = 1:amount_of_agents
            %             fid = fopen(strcat(filename,'/trajectory',num2str(itn),'.txt'),'rt');
            %             datacell = textscan(fid, '%f, %f', 'HeaderLines',5,'CollectOutput');
            %             fclose(fid)
            %
            if(testname == strcat(dirname,'/test_20181018_183757'))
                trajectory = importdata(strcat(filename,'/trajectory.txt'),',');
                
            else
                trajectory = importdata(strcat(filename,'/trajectory',num2str(itn),'.txt'),',');
            end
            
            
            for itl=3099:size(trajectory,1)
                
                if sqrt((trajectory(itl,1))^2+(trajectory(itl,2))^2)<2.5
                    
                    trajectory(itl+1:size(trajectory,1),:)=[];
                    break
                    
                end
            end
            
            if size(trajectory,1)>time_treshold(itt)
                
                made_it(it+1,itn) = 0;
                
            else
                made_it(it+1,itn) = 1;
            end
            
            
            
            if do_plot
                figure(1),
                plot(trajectory(1:3099,2),trajectory(1:3099,1),':','Color',[id_color_map(itn,:)],'LineWidth',2)
                plot(trajectory(3099,2),trajectory(3099,1),'o','Color',id_color_map(itn,:),'MarkerSize',20)
                plot(trajectory(3099:end,2),trajectory(3099:end,1),'Color',id_color_map(itn,:),'LineWidth',2)
                
                
            end
            
            %coverage calculation
            xy = trajectory + 10;
            xy(end,:)=xy(end-1,:);
            occMap_agent = robotics.BinaryOccupancyGrid(20,20,1);
            setOccupancy(occMap_agent,xy,1);
            OccMat_agent = occupancyMatrix(occMap_agent);
            OccMat_agent_all(:,:,itn) = OccMat_agent;
            coverage_per_agent(it+1,itn) = sum(OccMat_agent(:) == 1)/(numel(OccMat_agent)*accessable_area_percentage);
            
            
%             figure(5),show(occMap_agent),title(num2str(itn))
%             keyboard
            
            setOccupancy(occMap,xy,1)
            
            if made_it(it+1,itn) == 1
                setOccupancy(occMap_return,xy,1);
            end
            
            
            
            
        end
        
        %check uniqueness coverage per agent
        unique_coverage = zeros(size(OccMat_agent_all(:,:,1)));
        for itn = 1:amount_of_agents
            for itx = 1:size(OccMat_agent_all,1)
                for ity = 1:size(OccMat_agent_all,2)
                    
                    OccMat_agent_array = OccMat_agent_all(itx,ity,:);
                    if  OccMat_agent_all(itx,ity,itn) == true && numel(find(OccMat_agent_array==true))==1
                        unique_coverage(itx,ity) =1;
                    else
                        unique_coverage(itx,ity) =0;
                        
                    end
                    
                end
            end
            unique_coverage_per_agent(itn) = sum(unique_coverage(:)==1)/numel(unique_coverage);
        end
        unique_coverage_per_agent_tot(it+1) = mean(unique_coverage_per_agent);
        
        occMat = occupancyMatrix(occMap);
        occMat_return = occupancyMatrix(occMap_return);
        coverage_tot_return(it+1,1) = sum(occMat(:) == 1)/(numel(occMat)*accessable_area_percentage);
        coverage_tot_return(it+1,2) = sum(occMat_return(:) == 1)/(numel(occMat_return)*accessable_area_percentage);
        

        
        
        
        made_it(it+1,:)
        
        if do_plot
            figure(1)
            plot(8,8,'m*')
            plot(8,8,'mo','MarkerSize',90)
            
            keyboard
            hold off
            
        end
        
        
    end
    
    sum_made_it(:,itt) = sum(made_it,2);
%          figure(2),histogram(sum_made_it,'facecolor',map(1,:),'facealpha',.5,'edgecolor','none'), hold on
        
    
end




%%
close all


figure(2)

sum_made_it_it = sum(sum_made_it,1)/(amount_of_agents*(amount_of_environments+1))

bar(sum_made_it_it)
legendCell = cellstr(num2str(((time_treshold+100)/10)'))
set(gca,'xticklabel',legendCell)
xlabel('time threshold')
ylabel('percentage agent returned')


figure(3)

average_coverage_per_agent = mean(coverage_per_agent,2);

bar(mean([coverage_tot_return, average_coverage_per_agent, unique_coverage_per_agent_tot']))


% histdata = [];false
% for it=1:size_threshold
%
%     histdata = [histdata;histcounts(sum_made_it(:,it),0:amount_of_agents+1,'Normalization', 'probability')];
% end
% bar(0:amount_of_agents,histdata')
% legendCell = cellstr(num2str(((time_treshold+100)/10)'))
% legend(legendCell)
%
% xlabel('# agents returned')
% ylabel('occurance percentage')


%made_it = importdata(strcat(testname,'/made_it.txt'));

% success_rate = sum(made_it)/length(made_it)

keyboard
%% look at environment in detail
close all
% check_environment_number = 16

for check_environment_number=1:9

filename = strcat(testname,'/environment',num2str(check_environment_number));
img=255-imread(strcat(filename,'/environment.png'));
figure(3),
hold on,

xImg = linspace(-10, 10, size(img, 2));
yImg = linspace(-10,10, size(img, 1));
image(xImg, yImg, img, 'CDataMapping', 'scaled');
colormap(gray(256))

for itn=1:amount_of_agents

trajectory = importdata(strcat(filename,'/trajectory',num2str(itn),'.txt'),',');

%
%  for itl=3099:size(trajectory,1)
%
%                 if sqrt((trajectory(itl,1)-8)^2+(trajectory(itl,2)-8)^2)<2
%
%                     trajectory(itl+1:size(trajectory,1),:)=[];
%                     break
%
%                 end
%  end
plot(trajectory(1:3099,2),trajectory(1:3099,1),':','Color',[id_color_map(itn,:)],'LineWidth',2)
plot(trajectory(3099,2),trajectory(3099,1),'o','Color',id_color_map(itn,:),'MarkerSize',20)
plot(trajectory(3099:end,2),trajectory(3099:end,1),'Color',id_color_map(itn,:),'LineWidth',2)

plot(trajectory(3099,2),trajectory(3099,1),'o','Color',id_color_map(itn,:),'MarkerSize',20)
plot(0,0,'m*')

end
hold off

keyboard
end
% img_bin = im2bw(img);
% CC=bwconncomp(img_bin);
% 
% linearInd = sub2ind([200,200],100,100);
% index = 0;
% for it_temp=1:size(CC.PixelIdxList,1)
%     if ismember(linearInd,CC.PixelIdxList{it_temp})
%         index=it_temp;
%         break
%     end
% end
% img(CC.PixelIdxList{index}) = 150;
% figure, imshow(img,[0 255])
% keyboard

