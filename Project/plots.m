function plots(t,yt,ut,cpt,sbt,xbt,et,vt,flag,hxb)
 t = t./3; % Switches to hours 
 if (flag==1)
     figure; hold on;
     t1  = plot(t,yt(1,:),'LineWidth',1.5,'Color','b');
     t2  = plot(t,yt(2,:),'LineWidth',1.5,'Color','r');
     t3  = plot(t,yt(3,:),'LineWidth',1.5,'Color','g');
     ref = plot(t,24*ones(size(t)),'--','LineWidth',1,'Color',[0.5 0.5 0.5]);
     con = plot(t,22*ones(size(t)),'-.','LineWidth',1.2,'Color','k');
     plot(t,26*ones(size(t)),'-.','LineWidth',1.2,'Color','k');
     legend([t1,t2,t3,ref,con],{'Room 1 temperature','Room 2 temperature','Room 3 temperature','Target temperature','Temperature hard constraints'});
     title('Output tracking - building room temperature');
     xlabel('Hours'); ylabel('Temperature (C)');
     figure; hold on;
     u1  = stairs(t,ut(1,:),'LineWidth',1.5,'Color','b');
     u2  = stairs(t,ut(2,:),'LineWidth',1.5,'Color','r');
     u3  = stairs(t,ut(3,:),'LineWidth',1.5,'Color','g');
     con = plot(t,zeros(size(t)),'-.','LineWidth',1.5,'Color','k');
     plot(t,15*ones(size(t)),'-.','LineWidth',1.5,'Color','k');
     xlabel('Hours'); ylabel('Power Input (kW');
     legend([u1,u2,u3,con],{'Input Room 1','Input Room 2','Input Room 3','Inputs constraints'});
     title('MPC Control Inputs');
 end
 if (flag==2)
     figure; hold on;
     t1  = plot(t,yt(1,:),'LineWidth',1.5,'Color','b');
     t2  = plot(t,yt(2,:),'LineWidth',1.5,'Color','r');
     t3  = plot(t,yt(3,:),'LineWidth',1.5,'Color','g');
     ref = plot(t,24*ones(size(t)),'--','LineWidth',1,'Color',[0.5 0.5 0.5]);
     con = plot(t,22*ones(size(t)),'-.','LineWidth',1.2,'Color','k');
     plot(t,26*ones(size(t)),'-.','LineWidth',1.2,'Color','k');
     legend([t1,t2,t3,ref,con],{'Room 1 temperature','Room 2 temperature','Room 3 temperature','Target temperature','Temperature hard constraints'},'location','southeast');
     title('Output tracking - building room temperature');
     xlabel('Hours'); ylabel('Temperature (C)');
     figure; hold on;
     u1  = stairs(t,ut(1,:),'LineWidth',1.5,'Color','b');
     con = plot(t,zeros(size(t)),'-.','LineWidth',1.5,'Color','k');
     plot(t,15*ones(size(t)),'-.','LineWidth',1.5,'Color','k');
     p    = plot(t,10*cpt,'LineWidth',1,'Color','r');
     xlabel('Hours'); ylabel('Power Input (kW');
     legend([u1,con,p],{'Input Room 1','Inputs constraints','High/Low Price Time'});
     title('MPC Control Inputs');
 end
 if (flag==3)
     figure; hold on;
     t1  = plot(t,yt(1,:),'LineWidth',1.5,'Color','b');
     t2  = plot(t,yt(2,:),'LineWidth',1.5,'Color','r');
     t3  = plot(t,yt(3,:),'LineWidth',1.5,'Color','g');
     ref = plot(t,24*ones(size(t)),'--','LineWidth',1,'Color',[0.5 0.5 0.5]);
     con = plot(t,sbt(:,1),'-.','LineWidth',1.2,'Color','k');
     plot(t,sbt(:,2),'-.','LineWidth',1.2,'Color','k');
     legend([t1,t2,t3,ref,con],{'Room 1 temperature','Room 2 temperature','Room 3 temperature','Target temperature','Temperature hard constraints'},'location','northwest');
     title('Output tracking - building room temperature');
     xlabel('Hours'); ylabel('Temperature (C)');
     ylim([16,35]);
     figure; hold on;
     u1  = stairs(t,ut(1,:),'LineWidth',1.5,'Color','b');
     con = plot(t,zeros(size(t)),'-.','LineWidth',1.5,'Color','k');
     plot(t,15*ones(size(t)),'-.','LineWidth',1.5,'Color','k');
     p    = plot(t,10*cpt,'LineWidth',1,'Color','r');
     xlabel('Hours'); ylabel('Power Input (kW');
     legend([u1,con,p],{'Input Room 1','Inputs constraints','High/Low Price Time'});
     title('MPC Control Inputs');
 end
 if (flag==4)
     % Temperatures (output)
     figure; hold on;
     t1  = plot(t,yt(1,:),'LineWidth',1.5,'Color','b');
     t2  = plot(t,yt(2,:),'LineWidth',1.5,'Color','r');
     t3  = plot(t,yt(3,:),'LineWidth',1.5,'Color','g');
     ref = plot(t,24*ones(size(t)),'--','LineWidth',1,'Color',[0.5 0.5 0.5]);
     con = plot(t,sbt(:,1),'-.','LineWidth',1.2,'Color','k');
     plot(t,sbt(:,2),'-.','LineWidth',1.2,'Color','k');
     legend([t1,t2,t3,ref,con],{'Room 1 temperature','Room 2 temperature','Room 3 temperature','Target temperature','Temperature hard constraints'},'location','northwest');
     title('Output tracking - building room temperature');
     xlabel('Hours'); ylabel('Temperature (C)');
     ylim([16,35]);
     % Power (input)
     figure; hold on;
     u1  = stairs(t,ut(1,:),'LineWidth',1.5,'Color','b');
     con = plot(t,zeros(size(t)),'-.','LineWidth',1.5,'Color','k');
     plot(t,15*ones(size(t)),'-.','LineWidth',1.5,'Color','k');
     p    = plot(t,10*cpt,'LineWidth',1.5,'Color','r');
     xlabel('Hours'); ylabel('Power Input (kW');
     legend([u1,con,p],{'Input Room 1','Inputs constraints','High/Low Price Time'});
     title('MPC Control Inputs');
     
     % Battery's state and purchased power    
     figure;
     subplot(2,1,1); hold on;
     plot(t,et(1,:),'LineWidth',1.5,'Color','g');
     plot(t,50*cpt,'-','LineWidth',1.5,'Color','r');
     legend('Electrical Power Purchased','High/Low Price Time');
     ylabel('Purchased Power');
     xlabel('Hours');
     subplot(2,1,2); hold on;
     plot(t,vt(1,:),'-','LineWidth',1.5,'Color','b');
     plot(t,xbt(1,:),'-','LineWidth',1.5,'Color','c');
     plot(t,50*cpt,'-','LineWidth',1.5,'Color','r');
     plot(t,[20;-20].*ones(2,size(t,2)),'k-.','LineWidth',1.5);
     xlabel('Hours');
     ylabel('Storage');
     legend('Battery Output Power','Stored Electrical Power','High/Low Price Time','Battery Output Power Constraints');
 end
 if (flag==5)
       % Battery's state and purchased power    
     figure;
     subplot(2,1,1); hold on;
     plot(t,et(1,:),'LineWidth',1.5,'Color','g');
     plot(t,50*cpt,'-','LineWidth',1.5,'Color','r');
     legend('Electrical Power Purchased','High/Low Price Time');
     ylabel('Purchased Power');
     xlabel('Hours');
     subplot(2,1,2); hold on;
     plot(t,vt(1,:),'-','LineWidth',1.5,'Color','b');
     plot(t,xbt(1,:),'-','LineWidth',1.5,'Color','c');
     plot(t,50*cpt,'-','LineWidth',1.5,'Color','r');
     plot(t,[hxb;0].*ones(2,size(t,2)),'k-.','LineWidth',1.5);
     xlabel('Hours');
     ylabel('Storage');
     legend('Battery Output Power','Stored Electrical Power','High/Low Price Time','Battery Storage Constraints');
 end
end