import HasSelfPrivateNum;

public class ThreadA extends Thread{
    private HasSelfPrivateNum numRef;
    public ThreadA(HasSelfPrivateNum numRef){
        super();
        this.numRef = numRef;
    }
    @override
    public void run(){
        super.run();
        numRef.addI("a");
    }
}
