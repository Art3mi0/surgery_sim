#!/usr/bin/env python3

# This is a quick and dirty script for converting cls files generated from the tlx program to csv. It ignores the first line "NASA Task Data" and adds one more column with the total workload to the end and fills all of its columns with the value.

# Lines modifiable:
#	line 10 - number of participants
# line 14 - the starting participant number
# line 18 - the number of tests
# line 21 & 22 - in the chr() function, the starting test letter (would only change if the number of tests change, and if the starting test letter is not a)
# line 21 & 22 - the location of the files to convert and the destination of the converted files

if __name__ == '__main__':	

	# change this for the number of participants
	for i in range(2):
	# for some reason I could not just do str(i+1) in the long concatenation. It would run once, then give me an error despite working correctly for a single interation
		tmpNum = i+6 # change int to match the correct starting participant number
		
		# should not be changed unless less than 12 tests want to be converted
		for j in range(12):
			# should not have to change what is in chr() functions if you are doing a through l
			file2ConvertName = '/home/temo/experiment_data/all_tlx/p' + str(tmpNum) + '_' + chr(97 + j) + '.xls'
			convertedFileName = '/home/temo/experiment_data/tlx_converted/p' + str(tmpNum) + '_' + chr(97 + j) + '_tlx.csv' 
			file2Convert = open(file2ConvertName, 'r')
			lines = file2Convert.readlines() # opens file, saves lines, and closes
			file2Convert.close()
	
			convertedFile = open(convertedFileName, 'w')
	
			# initializes values that will be used later
			count = 0
			flag = False
			tmpLines = []
			tmpWL = 0
			count2 = 0
			
			# for my method, I am creating a string from the current line, modifying what is going in that string, and saving it to a list to be used by the writelines function at the end
			for line in lines:
				current_line = ""
				for i in line:
					# Ignores the first line and places comma in place of the special character that is not a space. It is not a space trust me
					if count >= 1:
						# Once it reaches the end only care about the number
						if flag:
							if i.isdigit() or i == '.':
								current_line += i
						elif i == '	':
							current_line += ','
						else:
							current_line += i
				
				# tw stands for total workload. csv files title their columns with the first line, but there was another title at the end with a single value. I add it to the column names in the first line and the value in every line for the rest of the column
				if flag:
					for j in range(len(tmpLines)):
						if j > 0:
							tmpLines[j] = tmpLines[j][0:len(tmpLines[j])-1] + ',' + current_line + "\n"
						else:
							tmpLines[j] = tmpLines[j][0:len(tmpLines[j])-1] + ",tw\n"
				# I checked the xls file and before the final line, there is an empty line between the final and second to last lines with values.
				elif len(current_line) == 1: 
					flag = True
				elif count >= 1:
					tmpLines.append(current_line)
				count += 1
			convertedFile.writelines(tmpLines)
			convertedFile.close()
