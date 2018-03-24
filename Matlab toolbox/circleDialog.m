function filter = circleDialog

    prompt = {'Enter thrshold value', 'Enter the number of biggest circles to draw'};
     num_lines =1;
     dlg_title = 'Enclosing Circle';
     defaultans = {'100', '5'};
     filter = inputdlg(prompt,dlg_title,num_lines,defaultans);  
     
end