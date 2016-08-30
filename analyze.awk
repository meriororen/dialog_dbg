BEGIN {
	get = 0;
	i = 0;
}

{ 
	gsub("^\\[0*", "", $1);
	gsub("\\]\|\<\|\>", ""); 

	if($5 == 25 && get == 0) 
	{
		get = 1; 
		i = 0; 
	}
	if ($5 == 25 && get == 1 && i > 0) 
	{
		get = 0; 
	}
	if (get == 1) 
	{ 
		arr[$5][++i] = $1; 
		linearr[$5,i] = NR;
	} 
} 

END { 

	for(m = 10; m < 30; m++) {
		if (m in arr) {
			for(t = 0; t < 100; t++) 
			{ 
				if (t in arr[m]) {
					switch (m) {
						case 10: r = "CLEARCHAR" 
							break
						case 11: r = "AUTHSEEDG" 
							break
						case 12: r = "AUTHTRY" 
							break
						case 13: r = "SEGSET" 
							break
						case 14: r = "AUTHSEEDR" 
							break
						case 15: r = "AUTHACK" 
							break
						case 16: r = "SEGACK" 
							break
						case 17: r = "GETVENDI" 
							break
						case 18: r = "GETVENDF" 
							break
						case 19: r = "GETDISCI" 
							break
						case 20: r = "UNCHARVR" 
							break
						case 21: r = "GETDEVII" 
							break
						case 22: r = "UNATTCDI" 
							break
						case 23: r = "VCARDEPR" 
							break
						case 24: r = "HEARTBT" 
							break
						case 25: r = "CONNOTIF" 
							break
						case 26: r = "DISNOTIF" 
							break
						case 27: r = "BLEMOD" 
							break
					}
					print linearr[m,t]": ["r"]["t"]->"arr[m][t]
				}
			} 
		}
		print "--"
	}
}
