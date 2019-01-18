%************************************************************************************
% Copyright (C) 2017                                                               %
% TETCOS, Bangalore. India                                                         %
%                                                                                  %
% Tetcos owns the intellectual property rights in the Product and its content.     %
% The copying, redistribution, reselling or publication of any or all of the       %
% Product or its content without express prior written consent of Tetcos is        %
% prohibited. Ownership and / or any other right relating to the software and all  %
% intellectual property rights therein shall remain at all times with Tetcos.      %
%                                                                                  %
% Author: 	Rahul								   %                                                                  % ---------------------------------------------------------------------------------%

function [A,B,C] = som_optimization(x,scount,num_cls,power,max_energy)
    	%  power: column vector of remaining power for each device
	% s_count is sensor_count
	% num_cls is number of clusters into which the sensors are grouped

	% The following code is for clustering using Self-Organizing Map Neural Network
	%    Clustering_Method     = 1         SOM using distance
    	%    Clustering Method     = 2         SOM using distance and remaining power
    	%    Clustering_Method     = 3         SOM and then iterative update of cluster heads

	Clustering_Method = 3;  	%change here for different algorithm
    	save som_clustering.mat
	%parameters that can be varied
	%selforgmap( dimension vector, coversteps , initial no of neighbor, initial topologyFnc,distanceFnc)
	
	neural_net = selforgmap([2 num_cls/2],100, 3,'gridtop','dist');  
	
	% Set epochs which works best for your setting
   
	neural_net.trainParam.epochs = 200;  
    	neural_net= train(neural_net,x.');
	output_net = neural_net(x.');
    	IDX=vec2ind(output_net);
	
  
 %Calculation of geometrical cluster centroids of clusters-
 
  C=zeros(num_cls,2);
  Geometrical_centroid=zeros(num_cls,2);
  n_clusters=zeros(num_cls,1);
  for i=1:1:scount
      Geometrical_centroid(IDX(i),:)= Geometrical_centroid(IDX(i),:)+x(i,:);
      n_clusters(IDX(i),1)=n_clusters(IDX(i),1)+1;
  end
  for i=1:1:num_cls
      Geometrical_centroid(i,:)=Geometrical_centroid(i,:)/n_clusters(i,1);
  end
  C=Geometrical_centroid;
  C=C(:);
  
  cl_count=zeros(1,num_cls);
  cl_dist=zeros(1,scount);
  
  if(Clustering_Method >= 2)				% only when method involves power
		cl_max_dist = zeros(1,num_cls);		% max distance of a point from cluster centre in each cluster
		cl_max_power = zeros(1,num_cls);	% max device power left in each cluster
        cl_min_power =zeros(1,num_cls);
	end
  
  % Calculation of Distance from each sensor to the centroid of its cluster and size of each cluster
	
		for index2=1:1:scount
			    cluster_index=IDX(index2);
                cl_count(cluster_index)=cl_count(cluster_index)+1;
				cl_dist(index2)= pdist([C(cluster_index),C(num_cls+cluster_index);x(index2,1),x(index2,2)],'euclidean'); % hard coded to be modified
				
				if(Clustering_Method >= 2)			% only when method involves power
					if cl_max_dist(cluster_index) < cl_dist(index2)
					   cl_max_dist(cluster_index) = cl_dist(index2);
					end

					if cl_max_power(cluster_index) < power(index2)
					   cl_max_power(cluster_index) = power(index2);
					end
				end
		end
		
	


	
iter_limit=1;   %  when clusters aren't being updated iteratively
if (Clustering_Method==3)
 %iter_limit should be increased as no of sensors in Network increase
iter_limit=100; 
end


% Cluster Head election
for i1=1:1:iter_limit
    cl_index= zeros(1,num_cls);
	c_head= zeros(1,num_cls);
	for cluster_index=1:1:num_cls
		if(Clustering_Method >= 2)	
			curr_cl_max_p = cl_max_power(cluster_index);
			curr_cl_max_d = cl_max_dist(cluster_index);
		end

		prev_dist_p = 100000;
		for index2=1:1:scount    
		 if IDX(index2)==cluster_index 
			 cl_index(cluster_index)=cl_index(cluster_index)+1;
			 curr_var = cl_dist(index2);
			 
			 if(Clustering_Method >= 2)		% only when method involves power
				curr_var = (curr_cl_max_p * cl_dist(index2)/(curr_cl_max_d + 0.0001)) - 10* power(index2);
			 end
			 
			 if(min(prev_dist_p,curr_var)== curr_var)
				c_head(cluster_index)=  index2;
				prev_dist_p= curr_var;
			 end

		 end

		end  
	end

  %Finding new cluster from the current cluster heads-
  if(Clustering_Method==3)
    cluster_count=zeros(1,num_cls);
	
    for i=1:1:scount
        min1=10000000;
        for j=1:1:num_cls
            if( pdist([x(c_head(1,j),1),x(c_head(1,j),2);x(i,1),x(i,2)],'euclidean')<min1)
                min1= pdist([x(c_head(j),1),x(c_head(j),2);x(i,1),x(i,2)],'euclidean');
                 IDX(i)=j;
            end
        end
        cluster_count(IDX(i))=cluster_count(IDX(i))+1;
        
    end
    cl_count=cluster_count;
	end
end

	% writing to log file about cluster head
 	fid = fopen('log.txt','a+');
 	for i=1:1:num_cls
 		fprintf(fid,'%d: (%d,%d)   ',c_head(i), x(c_head(i),1), x(c_head(i),2));
 	end
 	fprintf(fid,'\n');
 	fclose(fid);

   
   
% plot of the clusters and cluster heads
 % Uncomment the lines below to get a plot of cluster and their cluster_heads  
A1=['*' '+' '-' '.']
% figure();
% for i1=1:1:scount
%     plot(x(i1,1),x(i1,2),A1(IDX(i1)));
%     hold on;
% end
% for j1=1:1:num_cls
%     plot(x(c_head(j1),1),x(c_head(j1),2), 'g*');
%     hold on;
% end
% pause(0.5);
% close;

	

%Graph plotting of clusters ends here 
  
	A=c_head;    % contains the cluster head id's of each cluster
	B=IDX;	     % contains the cluster id's of each sensor
	C=cl_count;  % contains the size of each cluster

% 3D graph plotting starts here
tri = delaunay(x(:,1),x(:,2));
consumed = max_energy-power;
h = trisurf(tri,x(:,1),x(:,2),consumed);
%axis vis3d
x_max = max(x(:,1));
y_max = max(x(:,2));
p_max = max(consumed)+200;
axis([0 x_max 0 y_max 0 p_max]); 
xlabel('Sensor X position') 
ylabel('Sensor Y position') 
zlabel('Energy Consumed (mJ)')
lighting phong
shading interp
colorbar EastOutside
% 3D graph plotting ends here
end