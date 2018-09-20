function txt = myupdatefcn(empt,event_obj)
% Customizes text of data tips

pos = get(event_obj,'Position');
txt = {['Frequency: ',num2str(pos(1))],...
	      ['Magnitude: ',num2str(pos(2))]};