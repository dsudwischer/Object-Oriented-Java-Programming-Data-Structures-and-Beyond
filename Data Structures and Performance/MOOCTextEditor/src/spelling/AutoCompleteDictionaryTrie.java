package spelling;

import java.util.List;
import java.util.Queue;
import java.util.Set;
import java.util.Collection;
import java.util.HashMap;
import java.util.LinkedList;

/** 
 * An trie data structure that implements the Dictionary and the AutoComplete ADT
 * @author You
 *
 */

public class AutoCompleteDictionaryTrie implements  Dictionary, AutoComplete {

    private TrieNode root;
    private int size;
    

    public AutoCompleteDictionaryTrie()
	{
		root = new TrieNode();
	}
	
	
	/** Insert a word into the trie.
	 * For the basic part of the assignment (part 2), you should convert the 
	 * string to all lower case before you insert it. 
	 * 
	 * This method adds a word by creating and linking the necessary trie nodes 
	 * into the trie, as described outlined in the videos for this week. It 
	 * should appropriately use existing nodes in the trie, only creating new 
	 * nodes when necessary. E.g. If the word "no" is already in the trie, 
	 * then adding the word "now" would add only one additional node 
	 * (for the 'w').
	 * 
	 * @return true if the word was successfully added or false if it already exists
	 * in the dictionary.
	 */
	public boolean addWord(String word)
	{
	    word = word.toLowerCase();
	    TrieNode curNode = this.root;
	    boolean isNewWord = false;
	    for(int i = 0; i < word.length(); i++)
	    {
	    	char curChar = word.charAt(i);
	    	TrieNode nextNode = curNode.getChild(curChar);
	    	// Now, three cases can occur:
	    	// 1.: nextNode is null -> We should add the curChar to the trie
	    	// 2.: nextNode is not null and the word is not finished yet
	    	// 3.: nextNode is not null and the word is finished, i.e. this is the last
	    	// iteration of the loop
	    	
	    	// Case 1:
	    	if(nextNode == null)
	    	{
	    		isNewWord = true;
	    		nextNode = curNode.insert(curChar);
	    	}
	    	curNode = nextNode;
	    }
	    // At this point, curNode is corresponding to the last character in word
	    // So if curNode does not end a word, word is definitely new
	    if(!curNode.endsWord())
	    {
	    	curNode.setEndsWord(true);
	    	isNewWord = true;
	    }
	    // This will be false if and only if the word has been in the trie before
	    return isNewWord; 
	}
	
	/** 
	 * Return the number of words in the dictionary.  This is NOT necessarily the same
	 * as the number of TrieNodes in the trie.
	 */
	public int size()
	{
	    // Traverse the trie with breadth-first search and count the words
	    int numWords = 0;
	    // FIFO queue for BFS
	    Queue<TrieNode> queue = new LinkedList<TrieNode>();
	    queue.add(this.root);
	    // As long as there are nodes that have not been discovered, continue
	    while(!queue.isEmpty())
	    {
	    	TrieNode curNode = queue.poll();
	    	Set<Character> childrenCharacters = curNode.getValidNextCharacters();
	    	for(char c : childrenCharacters)
	    	{
	    		// If a child node ends a word, we increase the word count
	    		TrieNode childNode = curNode.getChild(c);
	    		if(childNode.endsWord())
	    		{
	    			numWords++;
	    		}
	    		// And we add the child to the queue
	    		queue.add(childNode);
	    	}
	    }
	    return numWords;
	}
	
	private List<String> getWordsAfterNode(TrieNode startNode, int maxWords)
	{
		// maxWords < 0 stands for no limit
		// Traverse the trie with breadth-first search
	    // FIFO queue for BFS
	    Queue<TrieNode> queue = new LinkedList<TrieNode>();
	    List<String> wordsAfterNode = new LinkedList<String>();
	    // If maxWords = 0, we are done
	    if(maxWords == 0)
	    {
	    	return wordsAfterNode;
	    }
	    // In this case, we have to actually find words
	    // Do not forget the startNode
	    else if(startNode.endsWord())
	    {
	    	wordsAfterNode.add(startNode.getText());
	    	if(maxWords == wordsAfterNode.size())
	    	{
	    		return wordsAfterNode;
	    	}
	    }
	    queue.add(startNode);
	    // As long as there are nodes that have not been discovered, continue
	    while(!queue.isEmpty())
	    {
	    	TrieNode curNode = queue.poll();
	    	Set<Character> childrenCharacters = curNode.getValidNextCharacters();
	    	for(char c : childrenCharacters)
	    	{
	    		// If a child node ends a word, we add it to the list
	    		TrieNode childNode = curNode.getChild(c);
	    		if(childNode.endsWord())
	    		{
	    			wordsAfterNode.add(childNode.getText());
	    			// If we have enough words, we return the list
	    			if(maxWords > 0 && wordsAfterNode.size() >= maxWords)
	    			{
	    				return wordsAfterNode;
	    			}
	    		}
	    		// And we add the child to the queue
	    		queue.add(childNode);
	    	}
	    }
	    return wordsAfterNode;
	}
	
	
	/** Returns whether the string is a word in the trie, using the algorithm
	 * described in the videos for this week. */
	@Override
	public boolean isWord(String s) 
	{
		s = s.toLowerCase();
		TrieNode stemFinishNode = findStem(s);
		return stemFinishNode != null && stemFinishNode.endsWord();
	}
	
	// A method to find the node that resembles a specific word stem
	// Note: This *is* case sensitve!
	private TrieNode findStem(String s)
	{
		TrieNode curNode = this.root;
		// Go along the way that is constructed by the characters in the string
		for(char c : s.toCharArray())
		{
			curNode = curNode.getChild(c);
			if(curNode == null)
			{
				return null;
			}
		}
		// If we reach this point, the complete stem is contained in the trie
		return curNode;
	}

	/** 
     * Return a list, in order of increasing (non-decreasing) word length,
     * containing the numCompletions shortest legal completions 
     * of the prefix string. All legal completions must be valid words in the 
     * dictionary. If the prefix itself is a valid word, it is included 
     * in the list of returned words. 
     * 
     * The list of completions must contain 
     * all of the shortest completions, but when there are ties, it may break 
     * them in any order. For example, if there the prefix string is "ste" and 
     * only the words "step", "stem", "stew", "steer" and "steep" are in the 
     * dictionary, when the user asks for 4 completions, the list must include 
     * "step", "stem" and "stew", but may include either the word 
     * "steer" or "steep".
     * 
     * If this string prefix is not in the trie, it returns an empty list.
     * 
     * @param prefix The text to use at the word stem
     * @param numCompletions The maximum number of predictions desired.
     * @return A list containing the up to numCompletions best predictions
     */@Override
     public List<String> predictCompletions(String prefix, int numCompletions) 
     {
    	 // This method should implement the following algorithm:
    	 // 1. Find the stem in the trie.  If the stem does not appear in the trie, return an
    	 //    empty list
    	 // 2. Once the stem is found, perform a breadth first search to generate completions
    	 //    using the following algorithm:
    	 //    Create a queue (LinkedList) and add the node that completes the stem to the back
    	 //       of the list.
    	 //    Create a list of completions to return (initially empty)
    	 //    While the queue is not empty and you don't have enough completions:
    	 //       remove the first Node from the queue
    	 //       If it is a word, add it to the completions list
    	 //       Add all of its child nodes to the back of the queue
    	 // Return the list of completions
    	 List<String> predictions = new LinkedList<String>();
    	 // First, we try and find the stem in the trie
    	 TrieNode stemEnd = this.findStem(prefix);
    	 if(stemEnd == null)
    	 {
    		 return predictions;
    	 }
         return getWordsAfterNode(stemEnd, numCompletions);
     }

 	// For debugging
 	public void printTree()
 	{
 		printNode(root);
 	}
 	
 	/** Do a pre-order traversal from this node down */
 	public void printNode(TrieNode curr)
 	{
 		if (curr == null) 
 			return;
 		
 		System.out.println(curr.getText());
 		
 		TrieNode next = null;
 		for (Character c : curr.getValidNextCharacters()) {
 			next = curr.getChild(c);
 			printNode(next);
 		}
 	}
 	

	
}