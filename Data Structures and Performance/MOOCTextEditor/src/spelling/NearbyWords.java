/**
 * 
 */
package spelling;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;


/**
 * @author UC San Diego Intermediate MOOC team
 *
 */
public class NearbyWords implements SpellingSuggest {
	// THRESHOLD to determine how many words to look through when looking
	// for spelling suggestions (stops prohibitively long searching)
	// For use in the Optional Optimization in Part 2.
	// Note: A threshold <= will be considered unlimited search volume.
	private static final int THRESHOLD = 1000000; 

	Dictionary dict;

	public NearbyWords (Dictionary dict) 
	{
		this.dict = dict;
	}

	/** Return the list of Strings that are one modification away
	 * from the input string.  
	 * @param s The original String
	 * @param wordsOnly controls whether to return only words or any String
	 * @return list of Strings which are nearby the original string
	 */
	public List<String> distanceOne(String s, boolean wordsOnly)
	{
		   List<String> retList = new LinkedList<String>();
		   insertions(s, retList, wordsOnly);
		   substitution(s, retList, wordsOnly);
		   deletions(s, retList, wordsOnly);
		   return retList;
	}

	
	/** Add to the currentList Strings that are one character mutation away
	 * from the input string.  
	 * @param s The original String
	 * @param currentList is the list of words to append modified words 
	 * @param wordsOnly controls whether to return only words or any String
	 * @return
	 */
	public void substitution(String s, List<String> currentList, boolean wordsOnly) {
		// for each letter in the s and for all possible replacement characters
		for(int index = 0; index < s.length(); index++){
			for(int charCode = (int)'a'; charCode <= (int)'z'; charCode++) {
				// use StringBuffer for an easy interface to permuting the 
				// letters in the String
				StringBuffer sb = new StringBuffer(s);
				sb.setCharAt(index, (char)charCode);

				// if the item isn't in the list, isn't the original string, and
				// (if wordsOnly is true) is a real word, add to the list
				addToList(s, currentList, wordsOnly, sb.toString());
			}
		}
	}
	
	/** Add to the currentList Strings that are one character insertion away
	 * from the input string.  
	 * @param s The original String
	 * @param currentList is the list of words to append modified words 
	 * @param wordsOnly controls whether to return only words or any String
	 * @return
	 */
	public void insertions(String s, List<String> currentList, boolean wordsOnly )
	{
		// for each letter in the s and for all possible additional (insertable) characters
		for(int index = 0; index <= s.length(); index++)
		{
			for(int charCode = (int)'a'; charCode <= (int)'z'; charCode++)
			{
				// use StringBuffer for an easy interface to insert the 
				// letters into the String
				StringBuffer sb = new StringBuffer(s);
				sb.insert(index, (char)charCode); 
				// if the item isn't in the list, isn't the original string, and
				// (if wordsOnly is true) is a real word, add to the list
				addToList(s, currentList, wordsOnly, sb.toString());
			}
		}
	}

	/** Add to the currentList Strings that are one character deletion away
	 * from the input string.  
	 * @param s The original String
	 * @param currentList is the list of words to append modified words 
	 * @param wordsOnly controls whether to return only words or any String
	 * @return
	 */
	public void deletions(String s, List<String> currentList, boolean wordsOnly )
	{
		// for each letter in the s
		for(int index = 0; index < s.length(); index++)
		{
			// use StringBuffer for an easy interface to delete 
			// a letter in the String
			StringBuffer sb = new StringBuffer(s);
			sb.deleteCharAt(index);
			// if the item isn't in the list, isn't the original string, and
			// (if wordsOnly is true) is a real word, add to the list
			addToList(s, currentList, wordsOnly, sb.toString());
		}
	}

	// Add modified String to list of words
	// Returns true if added
	private boolean addToList(String originalString, List<String> currentList, boolean wordsOnly,
			String toAdd)
	{
		// if the item isn't in the list, isn't the original string, and
		// (if wordsOnly is true) is a real word, add to the list
		if(!currentList.contains(toAdd) && 
				(!wordsOnly||dict.isWord(toAdd)) &&
				!originalString.equals(toAdd))
		{
			currentList.add(toAdd);
			return true;
		}
		return false;
	}
	
	/** Add to the currentList Strings that are one character deletion away
	 * from the input string.  
	 * @param word The misspelled word
	 * @param numSuggestions is the maximum number of suggestions to return 
	 * @return the list of spelling suggestions
	 */
	// Note:: All words are expected to be lower case
	@Override
	public List<String> suggestions(String word, int numSuggestions)
	{

		// initial variables
		Queue<String> queue = new LinkedList<String>();     // String to explore
		HashSet<String> visited = new HashSet<String>();   // to avoid exploring the same  
														   // string multiple times
		List<String> retList = new LinkedList<String>();   // words to return
		
		// Keep track of how many strings have been looked through.
		// If this number exceeds this.THRESHOLD, we will return the results.
		int numLookedThrough = 0;
		
		// insert first node
		queue.add(word);
		visited.add(word);
					
		// We start by exploring the words of distance one from word into the queue
		while(!queue.isEmpty() && retList.size() < numSuggestions)
		{
			// Pop the first element of the queue
			String curNodeString = queue.poll();
			// Build all distance one strings
			List<String> distanceOneStrings = distanceOne(curNodeString, false);
			// For each distance one string, add it to the visited set. If it has not been visited
			// before, also add it to the queue.
			// If it is a valid word, add it to the results.
			for(String s : distanceOneStrings)
			{
				boolean isNewString = visited.add(s);
				if(isNewString)
				{
					queue.add(s);
					if(this.dict.isWord(s))
					{
						retList.add(s);
					}
				}
				// If the desired number of suggestions has been reached, we return the results.
				// Also, if the threshold is > 0 and we have searched through more than THRESHOLD
				// strings, we return the results.
				if(retList.size() >= numSuggestions ||
						(this.THRESHOLD > 0 && ++numLookedThrough > this.THRESHOLD))
				{
					return retList;
				}
			}
		}
		
		return retList;

	}	

   public static void main(String[] args) {
	   //basic testing code to get started
	   String word = "i";
	   // Pass NearbyWords any Dictionary implementation you prefer
	   Dictionary d = new DictionaryHashSet();
	   DictionaryLoader.loadDictionary(d, "data/dict.txt");
	   NearbyWords w = new NearbyWords(d);
	   List<String> l = w.distanceOne(word, true);
	   System.out.println("One away word Strings for for \""+word+"\" are:");
	   System.out.println(l+"\n");

	   word = "tailo";
	   List<String> suggest = w.suggestions(word, 10);
	   System.out.println("Spelling Suggestions for \""+word+"\" are:");
	   System.out.println(suggest);
	   
   }

}
